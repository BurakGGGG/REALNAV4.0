#include "my_robot_hardware/diffdrive_system.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <cmath>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <array>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

namespace my_robot_hardware
{

hardware_interface::CallbackReturn DiffDriveSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    return hardware_interface::CallbackReturn::ERROR;

  // Ortak buffer'lar
  hw_pos_.assign(info_.joints.size(), 0.0);
  hw_vel_.assign(info_.joints.size(), 0.0);
  hw_cmd_.assign(info_.joints.size(), 0.0);

  // URDF ros2_control içindeki parametreleri okuyacağız
  if (info_.hardware_parameters.count("port")) port_ = info_.hardware_parameters.at("port");
  if (info_.hardware_parameters.count("baud")) baud_ = std::stoi(info_.hardware_parameters.at("baud"));
  if (info_.hardware_parameters.count("ticks_per_rev")) ticks_per_rev_ = std::stoi(info_.hardware_parameters.at("ticks_per_rev"));

  // İsteğe bağlı: max_wheel_rad_s parametresi
  if (info_.hardware_parameters.count("max_wheel_rad_s"))
  {
    try {
      max_wheel_rad_s_ = std::stod(info_.hardware_parameters.at("max_wheel_rad_s"));
    } catch (...) {
      max_wheel_rad_s_ = 20.0;
    }
  }

  // min_pwm parametresi
  if (info_.hardware_parameters.count("min_pwm"))
  {
    try {
      min_pwm_ = std::stoi(info_.hardware_parameters.at("min_pwm"));
    } catch (...) {
      min_pwm_ = 30;
    }
  }

  // cmd_deadband_rad_s parametresi
  if (info_.hardware_parameters.count("cmd_deadband_rad_s"))
  {
    try {
      cmd_deadband_rad_s_ = std::stod(info_.hardware_parameters.at("cmd_deadband_rad_s"));
    } catch (...) {
      cmd_deadband_rad_s_ = 0.05;
    }
  }

  // Encoder toplamları reset
  enc_left_total_ = 0;
  enc_right_total_ = 0;
  last_left_ticks_ = 0;
  last_right_ticks_ = 0;
  first_read_ = true;

  RCLCPP_INFO(
    rclcpp::get_logger("DiffDriveSystem"),
    "on_init: port=%s baud=%d ticks_per_rev=%d max_wheel_rad_s=%.2f",
    port_.c_str(), baud_, ticks_per_rev_, max_wheel_rad_s_);

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
  } else {
    RCLCPP_INFO(
      rclcpp::get_logger("DiffDriveSystem"),
      "STM32 connected on port %s at %d baud",
      port_.c_str(), baud_);
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
    // STM32 bağlı değilse hataya düşmeyelim, sadece komutları yok sayalım
    RCLCPP_WARN(
      rclcpp::get_logger("DiffDriveSystem"),
      "write() called but STM32 not connected, skipping commands");
    return hardware_interface::return_type::OK;
  }

  // Controller rad/s verir ama şu an DEBUG modda sabit PWM kullanıyoruz
  RCLCPP_INFO(
    rclcpp::get_logger("DiffDriveSystem"),
    "write() cmds rad_s: left=%.3f right=%.3f",
    hw_cmd_[0], hw_cmd_[1]);

  if (!send_wheel_rpm_(hw_cmd_[0], hw_cmd_[1]))
    return hardware_interface::return_type::ERROR;

  return hardware_interface::return_type::OK;
}

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

    // Make serial port fully raw and non-blocking:
    int fd = serial_->native_handle();

    // 1. O_NONBLOCK flag
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    // 2. Full raw mode — no output post-processing, no echo, no canonical
    struct termios tio;
    tcgetattr(fd, &tio);
    tio.c_iflag &= ~(IXON | IXOFF | IXANY | INLCR | ICRNL | IGNCR);
    tio.c_oflag &= ~OPOST;  // RAW output — \n stays as \n
    tio.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;
    tcsetattr(fd, TCSANOW, &tio);

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
  // STM32 sürekli satır basıyor varsayımı: en son gelen "ENC ..." satırını al
  std::string last_enc;
  bool got = false;

  for (int i = 0; i < 10; ++i) {
    std::string tmp;
    if (!read_line_(tmp)) break;
    // "ENC dL dR dt" formatı
    if (tmp.rfind("ENC", 0) == 0) {
      last_enc = tmp;
      got = true;
    }
  }
  if (!got) return false;

  // Örnek satır: "ENC -54 -224 100"
  std::istringstream ss(last_enc);
  std::string tag;
  int64_t dL = 0, dR = 0;
  uint64_t dt_ms = 0;

  ss >> tag >> dL >> dR >> dt_ms;
  if (ss.fail() || tag != "ENC") {
    return false;
  }

  // STM32 delta tick gönderiyor → biz toplam tick'e çeviriyoruz
  enc_left_total_  += dL;
  enc_right_total_ += dR;

  left  = enc_left_total_;
  right = enc_right_total_;

  RCLCPP_DEBUG(
    rclcpp::get_logger("DiffDriveSystem"),
    "ENC parsed: dL=%ld dR=%ld dt_ms=%lu totals: L=%ld R=%ld",
    static_cast<long>(dL), static_cast<long>(dR),
    static_cast<unsigned long>(dt_ms),
    static_cast<long>(enc_left_total_), static_cast<long>(enc_right_total_));

  return true;
}

bool DiffDriveSystem::send_wheel_rpm_(double left_rad_s, double right_rad_s)
{
  if (!connected_ || !serial_ || !serial_->is_open()) return false;

  // rad/s → PWM mapping: linear scale, clamped to [-255, +255]
  auto rad_s_to_pwm = [this](double rad_s) -> int32_t {
    if (std::abs(rad_s) < cmd_deadband_rad_s_) return 0;
    double ratio = rad_s / max_wheel_rad_s_;
    ratio = std::clamp(ratio, -1.0, 1.0);
    int32_t pwm = static_cast<int32_t>(ratio * 255.0);
    // Apply min_pwm threshold (minimum motor voltage to move)
    if (pwm != 0 && std::abs(pwm) < min_pwm_)
      pwm = (pwm > 0) ? min_pwm_ : -min_pwm_;
    return pwm;
  };

  const int32_t l_pwm = rad_s_to_pwm(left_rad_s);
  const int32_t r_pwm = rad_s_to_pwm(right_rad_s);

  RCLCPP_INFO(
    rclcpp::get_logger("DiffDriveSystem"),
    "PWM => L=%d  R=%d  (from L_rad_s=%.3f R_rad_s=%.3f)",
    l_pwm, r_pwm, left_rad_s, right_rad_s);

  boost::system::error_code ec;

  // Send L and R as SEPARATE writes with delay (STM32 needs time to parse)
  std::string cmd_l = "L " + std::to_string(l_pwm) + "\n";
  boost::asio::write(*serial_, boost::asio::buffer(cmd_l), ec);
  if (ec) return false;

  usleep(2000);  // 2ms delay between commands

  std::string cmd_r = "R " + std::to_string(r_pwm) + "\n";
  boost::asio::write(*serial_, boost::asio::buffer(cmd_r), ec);
  return !ec;
}

}  // namespace my_robot_hardware

PLUGINLIB_EXPORT_CLASS(my_robot_hardware::DiffDriveSystem, hardware_interface::SystemInterface)
