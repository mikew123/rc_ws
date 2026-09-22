#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <cerrno>
#include <fcntl.h>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <termios.h>
#include <vector>
#include <unistd.h>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robocolumbus_interfaces/msg/imu_cal.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>

using json = nlohmann::json;
using namespace std::chrono_literals;

class ImuGpsNode final : public rclcpp::Node
{
public:
  ImuGpsNode()
  : Node("imu_gps_node"), serial_fd_(-1), serial_port_index_(0)
  {
    serial_ports_ = declare_parameter<std::vector<std::string>>(
      "serial_ports", {
        "/dev/serial/by-id/usb-Waveshare_RP2040_Zero_E6635C469F25492A-if00",
        "/dev/serial/by-id/usb-Waveshare_RP2040_PiZero_E6635C469F25492A-if00"});

    json_msg_publisher_ = create_publisher<std_msgs::msg::String>("json_msg", 10);
    json_msg_subscription_ = create_subscription<std_msgs::msg::String>(
      "json_msg", 10, [](const std_msgs::msg::String::SharedPtr) {});
    imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    imu_cal_publisher_ = create_publisher<robocolumbus_interfaces::msg::ImuCal>("imu/cal", 10);
    gps_publisher_ = create_publisher<sensor_msgs::msg::NavSatFix>("gps_nav", 10);
    compass_publisher_ = create_publisher<std_msgs::msg::Float32>("cmp_azi", 10);

    serial_timer_ = create_wall_timer(10ms, std::bind(&ImuGpsNode::serial_timer_callback, this));
    startup_timer_ = create_wall_timer(2s, [this]() {
      publish_tts("IMU and GPS Node Started");
      startup_timer_->cancel();
    });

    RCLCPP_INFO(get_logger(), "ImuGpsNode started");
  }

  ~ImuGpsNode() override
  {
    close_serial();
  }

private:
  static constexpr speed_t kBaudRate = B1000000;
  static constexpr int kMinimumSatellites = 3;
  static constexpr std::array<double, 9> kGpsCovariance = {
    0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01};
  static constexpr std::array<double, 9> kImuCovariance = {
    0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001};

  void open_serial()
  {
    if (serial_ports_.empty()) {
      RCLCPP_ERROR(get_logger(), "No serial ports configured");
      return;
    }

    const auto & port = serial_ports_[serial_port_index_];
    const int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Failed to open serial port %s: %s", port.c_str(), std::strerror(errno));
      serial_port_index_ = (serial_port_index_ + 1) % serial_ports_.size();
      return;
    }

    termios settings{};
    if (tcgetattr(fd, &settings) != 0) {
      RCLCPP_ERROR(get_logger(), "Failed to read serial settings for %s: %s", port.c_str(),
        std::strerror(errno));
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
      RCLCPP_ERROR(get_logger(), "Failed to configure serial port %s: %s", port.c_str(),
        std::strerror(errno));
      close(fd);
      return;
    }

    tcflush(fd, TCIFLUSH);
    serial_fd_ = fd;
    receive_buffer_.clear();
    configured_ = false;
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

    if (!configured_) {
      send_json_command(json{{"cfg", {{"imu", true}, {"gps", true}, {"cmp", false}}}});
      configured_ = true;
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
        process_packet(line);
      }
    }

    if (receive_buffer_.size() > 16384) {
      RCLCPP_WARN(get_logger(), "Discarding oversized incomplete serial packet");
      receive_buffer_.clear();
    }
  }

  void process_packet(const std::string & line)
  {
    try {
      const json packet = json::parse(line);
      bool known = false;
      if (packet.contains("imu")) {
        publish_imu(packet.at("imu"));
        known = true;
      }
      if (packet.contains("gps")) {
        publish_gps(packet.at("gps"));
        known = true;
      }
      if (!known) {
        RCLCPP_WARN(get_logger(), "Unknown IMU/GPS packet: %s", line.c_str());
      }
    } catch (const json::exception & error) {
      RCLCPP_WARN(get_logger(), "Invalid IMU/GPS JSON: %s (%s)", line.c_str(), error.what());
    }
  }

  void publish_gps(const json & packet)
  {
    sensor_msgs::msg::NavSatFix message;
    message.header.stamp = get_clock()->now();
    message.header.frame_id = "gps_link";

    const bool status = packet.value("status", false);
    const int satellites = packet.value("siv", 0);
    if (status && satellites >= kMinimumSatellites) {
      message.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
      message.status.service = satellites;
      message.latitude = 1e-7 * packet.value("lat", 0.0);
      message.longitude = 1e-7 * packet.value("lon", 0.0);
      message.altitude = 1e-3 * packet.value("alt", 0.0);
    } else {
      message.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
      message.status.service = satellites;
      message.latitude = std::nan("");
      message.longitude = std::nan("");
      message.altitude = std::nan("");
    }

    message.position_covariance = kGpsCovariance;
    message.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    gps_publisher_->publish(message);
  }

  double median5_filter(double value, std::vector<double> & history)
  {
    history.push_back(value);
    if (history.size() > 5) {
      history.erase(history.begin());
    }
    if (history.size() < 5) {
      return value;
    }
    std::array<double, 5> sorted{};
    std::copy(history.begin(), history.end(), sorted.begin());
    std::sort(sorted.begin(), sorted.end());
    return sorted[2];
  }

  void publish_imu(const json & packet)
  {
    try {
      if (packet.contains("lacc")) {
        lacc_packet_ = packet.at("lacc");
      } else if (packet.contains("rvel")) {
        rvel_packet_ = packet.at("rvel");
      } else if (packet.contains("rvec")) {
        rvec_packet_ = packet.at("rvec");
      } else {
        RCLCPP_ERROR(get_logger(), "IMU JSON does not have lacc, rvel or rvec");
        return;
      }

      if (!lacc_packet_ || !rvel_packet_ || !rvec_packet_) {
        return;
      }

      const double i = rvec_packet_->value("i", 0.0);
      const double j = rvec_packet_->value("j", 0.0);
      const double k = rvec_packet_->value("k", 0.0);
      const double real = rvec_packet_->value("real", 1.0);

      double roll = std::atan2(2.0 * (real * i + j * k), 1.0 - 2.0 * (i * i + j * j));
      const double pitch_argument = 2.0 * (real * j - k * i);
      double pitch = std::asin(std::clamp(pitch_argument, -1.0, 1.0));
      double yaw = std::atan2(2.0 * (real * k + i * j), 1.0 - 2.0 * (j * j + k * k));

      roll = median5_filter(roll, roll_history_);
      pitch = median5_filter(pitch, pitch_history_);
      yaw = median5_filter(yaw + M_PI / 2.0, yaw_history_);
      if (yaw > M_PI) yaw -= 2.0 * M_PI;
      if (yaw < -M_PI) yaw += 2.0 * M_PI;
      std::swap(roll, pitch);
      pitch = -pitch;

      const double half_roll = roll / 2.0;
      const double half_pitch = pitch / 2.0;
      const double half_yaw = yaw / 2.0;
      const double cr = std::cos(half_roll);
      const double sr = std::sin(half_roll);
      const double cp = std::cos(half_pitch);
      const double sp = std::sin(half_pitch);
      const double cy = std::cos(half_yaw);
      const double sy = std::sin(half_yaw);

      sensor_msgs::msg::Imu message;
      message.header.stamp = get_clock()->now();
      message.header.frame_id = "imu_link";
      message.orientation.x = sr * cp * cy - cr * sp * sy;
      message.orientation.y = cr * sp * cy + sr * cp * sy;
      message.orientation.z = cr * cp * sy - sr * sp * cy;
      message.orientation.w = cr * cp * cy + sr * sp * sy;
      message.orientation_covariance = kImuCovariance;
      message.angular_velocity.x = rvel_packet_->value("x", 0.0);
      message.angular_velocity.y = rvel_packet_->value("y", 0.0);
      message.angular_velocity.z = rvel_packet_->value("z", 0.0);
      message.angular_velocity_covariance = kImuCovariance;
      message.linear_acceleration.x = lacc_packet_->value("x", 0.0);
      message.linear_acceleration.y = lacc_packet_->value("y", 0.0);
      message.linear_acceleration.z = lacc_packet_->value("z", 0.0);
      message.linear_acceleration_covariance = kImuCovariance;
      imu_publisher_->publish(message);

      const int rvel_calibration = rvel_packet_->value("stat", -1);
      const int rvec_calibration = rvec_packet_->value("stat", -1);
      const int lacc_calibration = lacc_packet_->value("stat", -1);
      robocolumbus_interfaces::msg::ImuCal calibration;
      calibration.rvel = static_cast<int8_t>(rvel_calibration);
      calibration.rvec = static_cast<int8_t>(rvec_calibration);
      calibration.lacc = static_cast<int8_t>(lacc_calibration);
      imu_cal_publisher_->publish(calibration);

      if (last_rvec_calibration_ != rvec_calibration) {
        last_rvec_calibration_ = rvec_calibration;
        publish_tts("IMU calibration status " + std::to_string(rvec_calibration));
        publish_calibration_status(rvec_calibration);
      }
      const auto now = std::chrono::steady_clock::now();
      if (now > next_calibration_status_) {
        next_calibration_status_ = now + 1s;
        publish_calibration_status(rvec_calibration);
      }

      lacc_packet_.reset();
      rvel_packet_.reset();
      rvec_packet_.reset();

      std_msgs::msg::Float32 compass;
      compass.data = static_cast<float>(yaw);
      compass_publisher_->publish(compass);
    } catch (const json::exception & error) {
      RCLCPP_ERROR(get_logger(), "IMU JSON exception: %s", error.what());
    }
  }

  void publish_calibration_status(int status)
  {
    publish_json(json{{"nav", {{"imu_cal_status", status}}}});
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
    if (serial_fd_ < 0) {
      return;
    }
    const std::string data = command.dump() + "\n";
    if (write(serial_fd_, data.data(), data.size()) < 0) {
      RCLCPP_WARN(get_logger(), "Failed to write to serial port: %s", std::strerror(errno));
    }
  }

  int serial_fd_;
  size_t serial_port_index_;
  bool configured_{false};
  std::vector<std::string> serial_ports_;
  std::string receive_buffer_;
  std::optional<json> lacc_packet_;
  std::optional<json> rvel_packet_;
  std::optional<json> rvec_packet_;
  std::vector<double> roll_history_;
  std::vector<double> pitch_history_;
  std::vector<double> yaw_history_;
  int last_rvec_calibration_{-1};
  std::chrono::steady_clock::time_point next_calibration_status_{};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr json_msg_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr json_msg_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<robocolumbus_interfaces::msg::ImuCal>::SharedPtr imu_cal_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr compass_publisher_;
  rclcpp::TimerBase::SharedPtr serial_timer_;
  rclcpp::TimerBase::SharedPtr startup_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuGpsNode>());
  rclcpp::shutdown();
  return 0;
}