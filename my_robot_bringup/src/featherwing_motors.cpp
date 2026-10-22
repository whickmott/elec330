#include <linux/i2c.h>
#include <hardware_interface/system_interface.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <stdexcept>
#include <vector>

namespace my_robot_bringup {
class FeatherWingMotors : public hardware_interface::SystemInterface {
  using Callback = hardware_interface::CallbackReturn;
  using Result = hardware_interface::return_type;
  int bus = -1;

  void send(std::initializer_list<uint8_t> values) {
    std::vector<uint8_t> bytes(values);
    if (::write(bus, bytes.data(), bytes.size()) != static_cast<ssize_t>(bytes.size()))
      throw std::runtime_error("FeatherWing I2C write failed");
  }

  void pin(int channel, int value) {
    uint8_t reg = 0x06 + 4 * channel;
    if (value == 4096) send({reg, 0, 0x10, 0, 0});
    else if (value == 0) send({reg, 0, 0, 0, 0x10});
    else send({reg, 0, 0, uint8_t(value & 0xff), uint8_t((value >> 8) & 0x0f)});
  }

  void motor(int index, double command) {
    const int pwm[]{8, 13}, forward[]{10, 11}, reverse[]{9, 12};
    if (!std::isfinite(command)) command = 0;
    command = std::clamp(command, -1.0, 1.0);
    pin(pwm[index], 0);
    pin(forward[index], command > 0 ? 4096 : 0);
    pin(reverse[index], command < 0 ? 4096 : 0);
    pin(pwm[index], static_cast<int>(std::abs(command) * 4095));
  }

  bool stop() noexcept {
    bool success = true;
    if (bus >= 0)
      for (int i = 0; i < 2; ++i)
        try { motor(i, 0); } catch (...) { success = false; }
    return success;
  }

public:
  ~FeatherWingMotors() override {
    stop();
    if (bus >= 0) close(bus);
  }

  Callback on_configure(const rclcpp_lifecycle::State &) override {
    if (info_.joints.size() != 2) return Callback::ERROR;
    bus = open("/dev/i2c-7", O_RDWR);
    if (bus < 0) {
      RCLCPP_ERROR(rclcpp::get_logger("FeatherWing"), "Cannot open /dev/i2c-7");
      return Callback::ERROR;
    }
    try {
      if (ioctl(bus, I2C_SLAVE, 0x60) < 0)
        throw std::runtime_error("Cannot select FeatherWing at 0x60");
      send({0x00, 0x30});
      send({0xfe, 3});
      send({0x01, 0x04});
      send({0x00, 0x20});
      usleep(10000);
      return stop() ? Callback::SUCCESS : Callback::ERROR;
    } catch (const std::exception &error) {
      stop();
      RCLCPP_ERROR(rclcpp::get_logger("FeatherWing"), "%s", error.what());
      return Callback::ERROR;
    }
  }

  Callback on_activate(const rclcpp_lifecycle::State &) override {
    for (const auto &joint : info_.joints) set_command(joint.name + "/pwm", 0.0);
    return stop() ? Callback::SUCCESS : Callback::ERROR;
  }

  Callback on_deactivate(const rclcpp_lifecycle::State &) override {
    return stop() ? Callback::SUCCESS : Callback::ERROR;
  }

  Callback on_shutdown(const rclcpp_lifecycle::State &) override {
    return stop() ? Callback::SUCCESS : Callback::ERROR;
  }

  Callback on_error(const rclcpp_lifecycle::State &) override {
    stop();
    return Callback::SUCCESS;
  }

  Callback on_cleanup(const rclcpp_lifecycle::State &) override {
    bool stopped = stop();
    if (bus >= 0) close(bus);
    bus = -1;
    return stopped ? Callback::SUCCESS : Callback::ERROR;
  }

    Result read(const rclcpp::Time &, const rclcpp::Duration &period) override {
    static int64_t zero[2]{};
    static bool have_zero[2]{};

    int fd = open("/dev/i2c-1", O_RDWR);
    if (fd < 0) {
      stop();
      return Result::ERROR;
    }

    try {
      for (int i = 0; i < 2; ++i) {
        uint16_t address = i == 0 ? 0x20 : 0x24;
        uint8_t settings[]{7, 127, 255, 255, 255, 128, 0, 0, 0, 0, 0, 0, 1};
        uint8_t reg = 3, bytes[4]{};
        i2c_msg messages[3]{
          {address, 0, sizeof(settings), settings},
          {address, 0, 1, &reg},
          {address, I2C_M_RD, 4, bytes}
        };
        i2c_rdwr_ioctl_data transaction{messages, 3};
        if (ioctl(fd, I2C_RDWR, &transaction) != 3)
          throw std::runtime_error("Encoder read failed");

        uint32_t raw = uint32_t(bytes[0]) << 24 | uint32_t(bytes[1]) << 16 |
                       uint32_t(bytes[2]) << 8 | bytes[3];
        int64_t count = raw;
        if (raw & 0x80000000U) count -= 0x100000000LL;

        const auto name = info_.joints[i].name;
        bool first_read = !have_zero[i];
        if (first_read) {
          zero[i] = count;
          have_zero[i] = true;
        }

        double previous = get_state(name + "/position");
        double scale = name == "base_left_wheel_joint" ? -0.1 : 0.1;
        double position = (count - zero[i]) * scale;
        double velocity = !first_read && std::isfinite(previous) && period.seconds() > 0
                        ? (position - previous) / period.seconds() : 0.0;

        set_state(name + "/position", position);
        set_state(name + "/velocity", velocity);
      }
      close(fd);
      return Result::OK;
    } catch (const std::exception &error) {
      close(fd);
      stop();
      RCLCPP_ERROR(rclcpp::get_logger("FeatherWing"), "%s", error.what());
      return Result::ERROR;
    }
  }

  Result write(const rclcpp::Time &, const rclcpp::Duration &) override {
    try {
      for (int i = 0; i < 2; ++i) motor(i, get_command(info_.joints[i].name + "/pwm"));
      return Result::OK;
    } catch (const std::exception &error) {
      stop();
      RCLCPP_ERROR(rclcpp::get_logger("FeatherWing"), "%s", error.what());
      return Result::ERROR;
    }
  }
};
}
PLUGINLIB_EXPORT_CLASS(my_robot_bringup::FeatherWingMotors, hardware_interface::SystemInterface)
