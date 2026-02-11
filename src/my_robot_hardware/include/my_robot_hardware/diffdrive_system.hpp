#pragma once
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <string>
#include <deque>
#include <memory>

#include <boost/asio.hpp>

namespace my_robot_hardware
{
class DiffDriveSystem : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  std::vector<double> hw_pos_, hw_vel_, hw_cmd_;

  // ===== UART / Serial config =====
  std::string port_{"/dev/serial0"};
  int baud_{115200};

  // ===== Encoder config =====
  int ticks_per_rev_{2048};
  bool encoder_is_delta_{false};

  int64_t last_left_ticks_{0};
  int64_t last_right_ticks_{0};
  bool first_read_{true};

  // ===== Serial communication =====
  std::unique_ptr<boost::asio::io_context> io_;
  std::unique_ptr<boost::asio::serial_port> serial_;
  std::string rx_buffer_;
  bool connected_{false};

  // ===== Helper functions =====
  bool connect_();
  void disconnect_();
  bool read_line_(std::string & line);
  bool read_encoders_(int64_t & left, int64_t & right);

  // Not: .cpp'de isim send_wheel_rpm_ ama biz rad/s gönderiyoruz, şimdilik böyle kalsın
  bool send_wheel_rpm_(double left_rad_s, double right_rad_s);

};  // class DiffDriveSystem
}  // namespace my_robot_hardware
