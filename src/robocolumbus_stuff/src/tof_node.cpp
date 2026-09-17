#include <array>
#include <chrono>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <memory>
#include <string>
#include <termios.h>
#include <vector>
#include <unistd.h>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robocolumbus_interfaces/msg/tof_dist.hpp>
#include <std_msgs/msg/string.hpp>

using json = nlohmann::json;
using namespace std::chrono_literals;

class TofNode final : public rclcpp::Node
{
public:
  TofNode()
  : Node("tof_node"), serial_fd_(-1), serial_port_index_(0)
  {
    serial_ports_ = declare_parameter<std::vector<std::string>>(
      "serial_ports", {
        "/dev/serial/by-id/usb-Waveshare_RP2040_Zero_45533065790A3B5A-if00",
        "/dev/serial/by-id/usb-Waveshare_RP2040_PiZero_45533065790A3B5A-if00"});

    json_msg_publisher_ = create_publisher<std_msgs::msg::String>("json_msg", 10);
    json_msg_subscription_ = create_subscription<std_msgs::msg::String>(
      "json_msg", 10, [](const std_msgs::msg::String::SharedPtr) {});
    tof_dist_publisher_ = create_publisher<robocolumbus_interfaces::msg::TofDist>("tof_dist", 10);

    serial_timer_ = create_wall_timer(10ms, std::bind(&TofNode::serial_timer_callback, this));
    startup_timer_ = create_wall_timer(2s, [this]() {
      publish_tts("Time of Flight Node Started");
      startup_timer_->cancel();
    });

    RCLCPP_INFO(get_logger(), "TofNode started");
  }

  ~TofNode() override
  {
    close_serial();
  }

private:
  static constexpr speed_t kBaudRate = B1000000;

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
      static constexpr std::array<const char *, 6> sensor_names = {
        "tof_fc", "tof_fl", "tof_fr", "tof_rc", "tof_rl", "tof_rr"};
      for (const char * sensor_name : sensor_names) {
        if (packet.contains(sensor_name)) {
          publish_sensor(sensor_name, packet.at(sensor_name));
          return;
        }
      }
      RCLCPP_WARN(get_logger(), "Unknown TOF packet: %s", line.c_str());
    } catch (const json::exception & error) {
      RCLCPP_WARN(get_logger(), "Invalid TOF JSON: %s (%s)", line.c_str(), error.what());
    }
  }

  void publish_sensor(const std::string & sensor_name, const json & sensor)
  {
    if (!sensor.contains("dist") || !sensor.at("dist").is_array() ||
      sensor.at("dist").size() != 8)
    {
      RCLCPP_WARN(get_logger(), "TOF packet for %s does not contain 8 distance rows", sensor_name.c_str());
      return;
    }

    robocolumbus_interfaces::msg::TofDist message;
    message.tof = sensor_name;
    const auto & distance_rows = sensor.at("dist");
    for (size_t row = 0; row < 8; ++row) {
      if (!distance_rows.at(row).is_array() || distance_rows.at(row).size() != 8) {
        RCLCPP_WARN(get_logger(), "TOF packet for %s does not contain 8 distances in row %zu",
          sensor_name.c_str(), row);
        return;
      }
      for (size_t column = 0; column < 8; ++column) {
        const auto & distance = distance_rows.at(row).at(column);
        if (!distance.is_number_integer()) {
          RCLCPP_WARN(get_logger(), "Non-integer distance in packet for %s", sensor_name.c_str());
          return;
        }
        message.dist[row * 8 + column] = distance.get<int16_t>();
      }
    }
    tof_dist_publisher_->publish(message);
  }

  void publish_tts(const std::string & text)
  {
    std_msgs::msg::String message;
    message.data = json{{"speaker", {{"tts", text}}}}.dump();
    json_msg_publisher_->publish(message);
  }

  int serial_fd_;
  size_t serial_port_index_;
  std::vector<std::string> serial_ports_;
  std::string receive_buffer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr json_msg_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr json_msg_subscription_;
  rclcpp::Publisher<robocolumbus_interfaces::msg::TofDist>::SharedPtr tof_dist_publisher_;
  rclcpp::TimerBase::SharedPtr serial_timer_;
  rclcpp::TimerBase::SharedPtr startup_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TofNode>());
  rclcpp::shutdown();
  return 0;
}