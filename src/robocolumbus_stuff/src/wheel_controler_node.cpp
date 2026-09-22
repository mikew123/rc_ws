#include <array>
#include <chrono>
#include <cmath>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <memory>
#include <string>
#include <termios.h>
#include <vector>
#include <unistd.h>

#include <geometry_msgs/msg/twist.hpp>
#include <nlohmann/json.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using json = nlohmann::json;
using namespace std::chrono_literals;

class WheelControllerNode final : public rclcpp::Node
{
public:
  WheelControllerNode()
  : Node("rc25_wheel_controler_node"), serial_fd_(-1), serial_port_index_(0),
    last_stamp_ms_(0), last_enc_(0), x_(0.0), y_(0.0), yaw_(0.0),
    last_steering_angle_(0.0), teleop_until_(0), kill_switch_(false),
    kill_switch_changed_(false), last_engine_status_()
  {
    serial_ports_ = declare_parameter<std::vector<std::string>>(
      "serial_ports", {
        "/dev/serial/by-id/usb-Waveshare_RP2040_Zero_E6625887D37C3E30-if00",
        "/dev/serial/by-id/usb-Waveshare_RP2040_PiZero_E6625887D37C3E30-if00"});

    json_msg_publisher_ = create_publisher<std_msgs::msg::String>("json_msg", 10);
    json_msg_subscription_ = create_subscription<std_msgs::msg::String>(
      "json_msg", 10, std::bind(&WheelControllerNode::json_msg_callback, this,
      std::placeholders::_1));
    wheel_odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("wheel_odom", 10);
    cmd_vel_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&WheelControllerNode::cmd_vel_callback, this,
      std::placeholders::_1));
    cmd_vel_teleop_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel/teleop", 10, std::bind(&WheelControllerNode::cmd_vel_teleop_callback, this,
      std::placeholders::_1));

    serial_timer_ = create_wall_timer(10ms,
      std::bind(&WheelControllerNode::serial_timer_callback, this));

    open_serial();
    if (serial_fd_ >= 0) {
      tcdrain(serial_fd_);
    }

    startup_timer_ = create_wall_timer(5s, [this]() {
      publish_tts("Wheel Controller Node Started");
      RCLCPP_INFO(get_logger(), "WheelControllerNode started");
      startup_timer_->cancel();
    });
  }

  ~WheelControllerNode() override
  {
    if (serial_fd_ >= 0) {
      send_json_command(json{{"wd", 100}, {"cv", {0, 0}}});
      send_json_command(json{{"mode", "bypass"}});
      tcdrain(serial_fd_);
    }
    close_serial();
  }

private:
  static constexpr speed_t kBaudRate = B1000000;
  static constexpr double kWheelBase = 0.490;
  static constexpr double kEncoderCountsPerMeter = 6000.0;
  static constexpr double kCoeffA = 0.2;
  static constexpr double kCoeffB = 0.04;
  static constexpr double kCoeffDa = 0.15;
  static constexpr double kCoeffDb = 0.075;

  void open_serial()
  {
    if (serial_ports_.empty()) {
      RCLCPP_ERROR(get_logger(), "No serial ports configured");
      return;
    }

    const auto & port = serial_ports_[serial_port_index_];
    const int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Failed to open serial port %s: %s", port.c_str(), std::strerror(errno));
      serial_port_index_ = (serial_port_index_ + 1) % serial_ports_.size();
      return;
    }

    termios settings{};
    if (tcgetattr(fd, &settings) != 0) {
      RCLCPP_ERROR(get_logger(), "Failed to read serial settings for %s: %s",
        port.c_str(), std::strerror(errno));
      close(fd);
      return;
    }
    cfmakeraw(&settings);
    cfsetispeed(&settings, kBaudRate);
    cfsetospeed(&settings, kBaudRate);
    settings.c_cflag |= CLOCAL | CREAD;
    settings.c_cflag &= ~CSTOPB;
    settings.c_cflag &= ~CRTSCTS;
    settings.c_cc[VMIN] = 0;
    settings.c_cc[VTIME] = 0;
    if (tcsetattr(fd, TCSANOW, &settings) != 0) {
      RCLCPP_ERROR(get_logger(), "Failed to configure serial port %s: %s",
        port.c_str(), std::strerror(errno));
      close(fd);
      return;
    }

    tcflush(fd, TCIFLUSH);
    serial_fd_ = fd;
    receive_buffer_.clear();
    send_json_command(json{{"pid", {kCoeffA, kCoeffB, kCoeffDa, kCoeffDb}}});
    send_json_command(json{{"mode", "cv"}});
    RCLCPP_INFO(get_logger(), "Opened serial port %s", port.c_str());
  }

  void close_serial()
  {
    if (serial_fd_ >= 0) {
      close(serial_fd_);
      serial_fd_ = -1;
    }
  }

  void serial_timer_callback()
  {
    if (serial_fd_ < 0) {
      open_serial();
      return;
    }

    std::array<char, 4096> buffer{};
    for (;;) {
      const ssize_t bytes_read = read(serial_fd_, buffer.data(), buffer.size());
      if (bytes_read > 0) {
        receive_buffer_.append(buffer.data(), static_cast<size_t>(bytes_read));
        process_complete_lines();
        continue;
      }
      if (bytes_read == 0 || errno == EAGAIN || errno == EWOULDBLOCK) {
        return;
      }
      RCLCPP_WARN(get_logger(), "Serial read failed: %s", std::strerror(errno));
      close_serial();
      return;
    }
  }

  void process_complete_lines()
  {
    size_t newline_position = 0;
    while ((newline_position = receive_buffer_.find('\n')) != std::string::npos) {
      std::string line = receive_buffer_.substr(0, newline_position);
      receive_buffer_.erase(0, newline_position + 1);
      if (!line.empty() && line.back() == '\r') {
        line.pop_back();
      }
      if (!line.empty()) {
        try {
          process_serial_packet(json::parse(line));
        } catch (const json::exception &) {
          // Ignore non-JSON serial lines, matching the Python node behavior.
        }
      }
    }
    if (receive_buffer_.size() > 16384) {
      receive_buffer_.clear();
    }
  }

  void process_serial_packet(const json & packet)
  {
    if (packet.contains("odom")) {
      process_wheel_odom(packet.at("odom"));
    }
    if (packet.contains("status")) {
      process_engine_status(packet.at("status").get<std::string>());
    }
  }

  void process_engine_status(const std::string & status)
  {
    if (status != last_engine_status_) {
      publish_json(json{{"engine", {{"status", status}}}});
      last_engine_status_ = status;
    }
  }

  void process_wheel_odom(const json & odom)
  {
    const auto current_time = get_clock()->now();
    const uint32_t stamp_ms = odom.at("stamp").get<uint32_t>();
    const int32_t enc = odom.at("enc").get<int32_t>();
    const double steer = -odom.at("steer").get<double>();
    const double dt = static_cast<double>(stamp_ms - last_stamp_ms_) * 1e-3;
    last_stamp_ms_ = stamp_ms;
    const int32_t delta_enc = enc - last_enc_;
    last_enc_ = enc;
    if (dt > 0.1 || dt <= 0.0) {
      return;
    }

    const double distance = static_cast<double>(delta_enc) / kEncoderCountsPerMeter;
    const double velocity = distance / dt;
    double angular_velocity = velocity * std::tan(steer) / kWheelBase;
    angular_velocity /= 2.4;
    yaw_ += angular_velocity * dt;
    if (yaw_ > M_PI) yaw_ -= 2.0 * M_PI;
    if (yaw_ < -M_PI) yaw_ += 2.0 * M_PI;
    x_ += distance * std::cos(yaw_);
    y_ += distance * std::sin(yaw_);

    nav_msgs::msg::Odometry message;
    message.header.stamp = current_time;
    message.header.frame_id = "odom";
    message.child_frame_id = "base_footprint";
    message.pose.pose.position.x = x_;
    message.pose.pose.position.y = y_;
    message.pose.pose.orientation.z = std::sin(yaw_ / 2.0);
    message.pose.pose.orientation.w = std::cos(yaw_ / 2.0);
    message.pose.covariance = {1e-12, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 1e-12, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 1e-12, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 1e-12, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 1e-12, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 1e-12};
    message.twist.twist.linear.x = velocity;
    message.twist.twist.angular.z = angular_velocity;
    message.twist.covariance = {0.001, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 1e-12, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 1e-12, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 1e-12, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 1e-12, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 1e-12};
    wheel_odom_publisher_->publish(message);
  }

  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr message)
  {
    double linear_x = message->linear.x;
    double angular_z = message->angular.z;
    if (kill_switch_) {
      if (kill_switch_changed_) {
        kill_switch_changed_ = false;
        RCLCPP_INFO(get_logger(), "Kill switch is active - stopping");
      }
      linear_x = 0.0;
      angular_z = 0.0;
    }
    cmd_vel_to_wheels(linear_x, angular_z, false);
  }

  void cmd_vel_teleop_callback(const geometry_msgs::msg::Twist::SharedPtr message)
  {
    cmd_vel_to_wheels(message->linear.x, message->angular.z, true);
  }

  void cmd_vel_to_wheels(double linear_x, double angular_z, bool teleop)
  {
    const auto now = get_clock()->now();
    if (teleop) {
      teleop_until_ = now.nanoseconds() + 1000000000LL;
    } else if (now.nanoseconds() < teleop_until_) {
      return;
    }

    angular_z *= 2.4;
    const double steering_angle = std::abs(linear_x) > 0.01 ?
      -std::atan(kWheelBase * angular_z / linear_x) : last_steering_angle_;
    last_steering_angle_ = steering_angle;
    const double wheel_velocity = linear_x;
    send_json_command(json{{"wd", 1000}, {"cv", {wheel_velocity == 0.0 ? 0.0 : wheel_velocity,
      wheel_velocity == 0.0 ? 0.0 : steering_angle}}});
  }

  void json_msg_callback(const std_msgs::msg::String::SharedPtr message)
  {
    try {
      const json data = json::parse(message->data);
      if (data.contains("nav") && data.at("nav").contains("kill")) {
        const bool kill = data.at("nav").at("kill").get<bool>();
        if (kill != kill_switch_) {
          kill_switch_changed_ = true;
          RCLCPP_INFO(get_logger(), "Kill switch is %s", kill ? "true" : "false");
        }
        kill_switch_ = kill;
      }
    } catch (const json::exception &) {
      RCLCPP_WARN(get_logger(), "Ignoring invalid json_msg payload");
    }
  }

  void publish_tts(const std::string & text)
  {
    publish_json(json{{"speaker", {{"tts", text}}}});
  }

  void publish_json(const json & value)
  {
    std_msgs::msg::String message;
    message.data = value.dump();
    json_msg_publisher_->publish(message);
  }

  void send_json_command(const json & command)
  {
    if (serial_fd_ < 0) return;
    const std::string line = command.dump() + '\n';
    const ssize_t written = write(serial_fd_, line.data(), line.size());
    if (written < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
      RCLCPP_WARN(get_logger(), "Failed to write to serial: %s", std::strerror(errno));
    }
  }

  int serial_fd_;
  size_t serial_port_index_;
  std::vector<std::string> serial_ports_;
  std::string receive_buffer_;
  uint32_t last_stamp_ms_;
  int32_t last_enc_;
  double x_, y_, yaw_, last_steering_angle_;
  int64_t teleop_until_;
  bool kill_switch_, kill_switch_changed_;
  std::string last_engine_status_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr json_msg_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr json_msg_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_teleop_subscription_;
  rclcpp::TimerBase::SharedPtr serial_timer_;
  rclcpp::TimerBase::SharedPtr startup_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelControllerNode>());
  rclcpp::shutdown();
  return 0;
}