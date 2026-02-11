#include "my_robot_hardware/diffdrive_system.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <cmath>
#include <sstream>
#include <iomanip>
#include <array>
#include <fcntl.h>
#include <unistd.h>

namespace my_robot_hardware
{

hardware_interface::CallbackReturn DiffDriveSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    return hardware_interface::CallbackReturn::ERROR;

  hw_pos_.assign(info_.joints.size(), 0.0);
  hw_vel_.assign(info_.joints.size(), 0.0);
  hw_cmd_.assign(info_.joints.size(), 0.0);

  // URDF ros2_control içindeki parametreleri okuyacağız
  if (info_.hardware_parameters.count("port")) port_ = info_.hardware_parameters["port"];
  if (info_.hardware_parameters.count("baud")) baud_ = std::stoi(info_.hardware_parameters["baud"]);
  if (info_.hardware_parameters.count("ticks_per_rev")) ticks_per_rev_ = std::stoi(info_.hardware_parameters["ticks_per_rev"]);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffDriveSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> states;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    states.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_pos_[i]);
    states.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_vel_[i]);
  }
  return states;
}

std::vector<hardware_interface::CommandInterface> DiffDriveSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> cmds;
  for (size_t i = 0; i < info_.joints.size(); ++i)
    cmds.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_cmd_[i]);
  return cmds;
}

hardware_interface::CallbackReturn DiffDriveSystem::on_activate(
  const rclcpp_lifecycle::State &)
{
  connected_ = connect_();

  if (!connected_) {
    RCLCPP_WARN(
      rclcpp::get_logger("DiffDriveSystem"),
      "STM32 not connected. Running in dummy mode."
    );
  }

  first_read_ = true;
  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn DiffDriveSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  // güvenlik: durdur
  send_wheel_rpm_(0.0, 0.0);
  disconnect_();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffDriveSystem::read(const rclcpp::Time &, const rclcpp::Duration & period)
{
  int64_t lt, rt;
  if (!read_encoders_(lt, rt)) {
  // Encoder verisi gelmiyorsa (STM32 takılı değil / henüz veri yok):
  // Sistem çökmesin, sadece hızları 0 kabul et.
  hw_vel_[0] = 0.0;
  hw_vel_[1] = 0.0;
  return hardware_interface::return_type::OK;
}


  if (first_read_)
  {
    last_left_ticks_ = lt;
    last_right_ticks_ = rt;
    first_read_ = false;
    return hardware_interface::return_type::OK;
  }

  const double dt = period.seconds();
  if (dt <= 0.0) return hardware_interface::return_type::OK;

  const double ticks_to_rad = (2.0 * M_PI) / static_cast<double>(ticks_per_rev_);

  const int64_t dlt = lt - last_left_ticks_;
  const int64_t drt = rt - last_right_ticks_;
  last_left_ticks_ = lt;
  last_right_ticks_ = rt;

  const double left_delta = static_cast<double>(dlt) * ticks_to_rad;
  const double right_delta = static_cast<double>(drt) * ticks_to_rad;

  // joint sırası: URDF'de ros2_control içinde hangi sıradaysa ona göre.
  // Senin URDF: left_wheel_joint, right_wheel_joint
  hw_pos_[0] += left_delta;
  hw_pos_[1] += right_delta;

  hw_vel_[0] = left_delta / dt;
  hw_vel_[1] = right_delta / dt;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DiffDriveSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
   if (!connected_) {
  return hardware_interface::return_type::OK;
}

  // Controller zaten rad/s verir. STM32'ye rad/s gönderiyoruz.
  if (!send_wheel_rpm_(hw_cmd_[0], hw_cmd_[1]))
    return hardware_interface::return_type::ERROR;

  return hardware_interface::return_type::OK;
}
  

// ==== ŞİMDİLİK BOŞ (DOLDURACAĞIZ) ====
// Şu an sadece “derlensin ve plugin yüklensin” diye true dönüyor.

bool DiffDriveSystem::connect_()
{
  try {
    io_ = std::make_unique<boost::asio::io_context>();
    serial_ = std::make_unique<boost::asio::serial_port>(*io_);

    serial_->open(port_);
    serial_->set_option(boost::asio::serial_port_base::baud_rate(baud_));
    serial_->set_option(boost::asio::serial_port_base::character_size(8));
    serial_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    serial_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    serial_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

    int flags = fcntl(serial_->native_handle(), F_GETFL, 0);
    fcntl(serial_->native_handle(), F_SETFL, flags | O_NONBLOCK);

    rx_buffer_.clear();
    connected_ = true;
    return true;
  } catch (...) {
    connected_ = false;
    return false;
  }
}

void DiffDriveSystem::disconnect_()
{
  try {
    if (serial_ && serial_->is_open()) {
      serial_->close();
    }
  } catch (...) {}
  connected_ = false;
}

bool DiffDriveSystem::read_line_(std::string & line)
{
  if (!connected_) return false;

  if (!serial_ || !serial_->is_open()) return false;

  boost::system::error_code ec;
  std::array<char, 256> buf{};
  const size_t n = serial_->read_some(boost::asio::buffer(buf), ec);

  if (ec) {
    // veri yoksa normal
    if (ec == boost::asio::error::would_block || ec == boost::asio::error::try_again) return false;
    return false;
  }
  if (n == 0) return false;

  rx_buffer_.append(buf.data(), n);

  const auto pos = rx_buffer_.find('\n');
  if (pos == std::string::npos) return false;

  line = rx_buffer_.substr(0, pos);
  rx_buffer_.erase(0, pos + 1);

  if (!line.empty() && line.back() == '\r') line.pop_back();
  return true;
}

bool DiffDriveSystem::read_encoders_(int64_t & left, int64_t & right)
{
  // STM32 sürekli satır basıyor varsayımı: en son gelen "E ..." satırını al
  std::string last_e;
  bool got = false;

  for (int i = 0; i < 10; ++i) {
    std::string tmp;
    if (!read_line_(tmp)) break;
    if (!tmp.empty() && tmp[0] == 'E') {
      last_e = tmp;
      got = true;
    }
  }
  if (!got) return false;

  std::istringstream ss(last_e);
  char tag;
  ss >> tag >> left >> right;
  return !ss.fail();
}

bool DiffDriveSystem::send_wheel_rpm_(double left_rad_s, double right_rad_s)
{
  // Not: isim rpm_ ama burada rad/s gönderiyoruz (isim sonra düzeltilebilir)
  if (!connected_ || !serial_ || !serial_->is_open()) return false;

  std::ostringstream out;
  out << "V " << std::fixed << std::setprecision(4) << left_rad_s << " " << right_rad_s << "\n";
  const std::string s = out.str();

  boost::system::error_code ec;
  boost::asio::write(*serial_, boost::asio::buffer(s), ec);
  return !ec;
}


}  // namespace my_robot_hardware

PLUGINLIB_EXPORT_CLASS(my_robot_hardware::DiffDriveSystem, hardware_interface::SystemInterface)
