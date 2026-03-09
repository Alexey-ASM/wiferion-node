#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <string>
#include <thread>

using namespace std::chrono_literals;

class CanReaderNode : public rclcpp::Node 
{
public:
  CanReaderNode() : rclcpp::Node("can_reader_node") 
  {
    batteryState_.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIFE;

    batteryState_.cell_voltage.resize(4, std::numeric_limits<double>::quiet_NaN());
    batteryState_.cell_temperature.resize(3, std::numeric_limits<double>::quiet_NaN());

    batteryState_.present = true;

    std::string can_interface = this->declare_parameter<std::string>("can_interface", "can0");

    publisher_ = this->create_publisher<sensor_msgs::msg::BatteryState>("battery/state", 10);

    socket_fd_ = open_can_socket(can_interface);
    running_.store(true);
    reader_thread_ = std::thread(&CanReaderNode::read_loop, this);

    timer_ = create_wall_timer(1000ms, std::bind(&CanReaderNode::publish_state, this));

    RCLCPP_INFO(this->get_logger(), "Reading CAN from %s and publishing to %s", can_interface.c_str(), publisher_->get_topic_name());
  }

  ~CanReaderNode() override 
  {
    running_.store(false);
    if (socket_fd_ >= 0) 
    {
      close(socket_fd_);
      socket_fd_ = -1;
    }
    if (reader_thread_.joinable()) 
    {
      reader_thread_.join();
    }
  }

private:
  int open_can_socket(const std::string &ifname) 
  {
    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) 
    {
        throw std::runtime_error(std::string("socket() failed: ") + std::strerror(errno));
    }

    struct ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::snprintf(ifr.ifr_name, IFNAMSIZ, "%s", ifname.c_str());

    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) 
    {
      close(s);
      throw std::runtime_error(std::string("ioctl(SIOCGIFINDEX) failed for ") + ifname + ": " + std::strerror(errno));
    }

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) 
    {
      close(s);
      throw std::runtime_error(std::string("bind() failed for ") + ifname + ": " + std::strerror(errno));
    }

    return s;
  }

  void read_loop() 
  {
    while (running_.load() && rclcpp::ok()) 
    {
      struct can_frame frame;
      const int nbytes = static_cast<int>(read(socket_fd_, &frame, sizeof(frame)));

      if (nbytes < 0) 
      {
        if (errno == EINTR) 
        {
          continue;
        }
        
        RCLCPP_WARN(this->get_logger(), "CAN read error: %s", std::strerror(errno));
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        continue;
      }

      if (nbytes != static_cast<int>(sizeof(struct can_frame))) 
      {
        RCLCPP_WARN(this->get_logger(), "Incomplete CAN frame read: %d", nbytes);
        continue;
      }

      switch(frame.can_id)
      {
          case 0x100:
              batteryState_.percentage = frame.data[3] * 0.01;
              leaderBatteryStatus_ = frame.data[1];
              failStatus1_ = frame.data[0];
              batteryState_.cell_temperature[2] = (frame.data[5] * 0x100 + frame.data[6]) * 0.1;
              break;
          case 0x101:
              batteryState_.current = int16_t(frame.data[0] * 0x100 + frame.data[1]) * 0.01;
              batteryState_.voltage = (frame.data[4] * 0x100 + frame.data[5]) * 0.001;
              failStatus2_ = frame.data[6];
              break;
          case 0x103:
              batteryState_.charge = int16_t(frame.data[4] * 0x100 + frame.data[5]) * 0.01;
              batteryState_.capacity = int16_t(frame.data[2] * 0x100 + frame.data[3]) * 0.01;
              batteryState_.design_capacity = int16_t(frame.data[0] * 0x100 + frame.data[1]) * 0.01;
              fetStatus_ = frame.data[6];
              break;
          case 0x110:
              batteryState_.cell_voltage[0] = (frame.data[0] * 0x100 + frame.data[1]) * 0.001;
              batteryState_.cell_voltage[1] = (frame.data[4] * 0x100 + frame.data[5]) * 0.001;
              break;
          case 0x111:
              batteryState_.temperature = (frame.data[0] * 0x100 + frame.data[1]) * 0.1;
              batteryState_.cell_temperature[0] = (frame.data[0] * 0x100 + frame.data[1]) * 0.1;
              batteryState_.cell_temperature[1] = (frame.data[4] * 0x100 + frame.data[5]) * 0.1;
              break;
          case 0x120:
              batteryState_.cell_voltage[2] = (frame.data[0] * 0x100 + frame.data[1]) * 0.001;
              batteryState_.cell_voltage[3] = (frame.data[4] * 0x100 + frame.data[5]) * 0.001;
              break;
      }
    }
  }

  void publish_state() 
  {
    batteryState_.header.stamp = now(); 
    batteryState_.header.frame_id = "battery_frame";

    if(fetStatus_ & FET_CHARGE_ON) 
    {
      batteryState_.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
    } 
    else if(leaderBatteryStatus_ == static_cast<std::underlying_type_t<LeaderBatteryStatus>>(LeaderBatteryStatus::LEADER_BATTERY_STATUS_FULL)) 
    {
      batteryState_.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_FULL;
    }
    else if(fetStatus_ & FET_DISCHARGE_ON) 
    {
      batteryState_.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
    } 
    else 
    {
      batteryState_.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING;
    }

    //if(failStatus1_ & FAIL_STATUS1_OVERCURRENT_DISCHARGE1 || failStatus1_ & FAIL_STATUS1_OVERCURRENT_DISCHARGE2 || failStatus1_ & FAIL_STATUS1_OVERCURRENT_DISCHARGE3) 
    //{
    //  batteryState_.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_OVERCURRENT;
    //} 
    //else if(failStatus1_ & FAIL_STATUS1_OVERCHARGE || failStatus2_ & FAIL_STATUS2_OVERCHARGE) 
    //{
    //  batteryState_.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_OVERHEAT;
    //} 
    //else 
    if(failStatus1_ & FAIL_STATUS1_OVER_TEMPERATURE_DISCHARGE || failStatus2_ & FAIL_STATUS2_OVER_TEMPERATURE_CHARGE) 
    {
      batteryState_.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_OVERHEAT;
    } 
    else if(failStatus1_ & FAIL_STATUS1_LOW_VOLTAGE || failStatus2_ & FAIL_STATUS2_DEEP_DISCHARGE) 
    {
      batteryState_.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_DEAD;
    } 
    else if(((failStatus1_ & ~FAIL_STATUS1_FULLY_CHARGED) == 0) && ((failStatus2_ & ~FAIL_STATUS2_FULLY_CHARGED) == 0)) 
    {
      batteryState_.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
    } 
    else
    {
      batteryState_.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
    }

    publisher_->publish(batteryState_);
  }

  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr publisher_;
  std::thread reader_thread_;
  std::atomic<bool> running_{false};
  int socket_fd_{-1};

  rclcpp::TimerBase::SharedPtr timer_;

  sensor_msgs::msg::BatteryState batteryState_;

  enum FETStatus 
  {
    FET_DISCHARGE_ON = 1 << 0,
    FET_CHARGE_ON    = 1 << 1,
  };

  enum class LeaderBatteryStatus 
  {
    LEADER_BATTERY_STATUS_FULL = 0x03,
    // Add more status flags as needed
  };

  enum FailStatus1 
  {
    FAIL_STATUS1_OVERCURRENT_DISCHARGE1 = 1 << 0,
    FAIL_STATUS1_OVERCURRENT_DISCHARGE2 = 1 << 1,
    FAIL_STATUS1_OVERCHARGE = 1 << 2,
    FAIL_STATUS1_OVERCURRENT_CHARGE = 1 << 3,
    FAIL_STATUS1_OVER_TEMPERATURE_DISCHARGE = 1 << 4, 
    FAIL_STATUS1_LOW_VOLTAGE = 1 << 5,
    FAIL_STATUS1_FULLY_CHARGED = 1 << 6,
    FAIL_STATUS1_OVERCURRENT_DISCHARGE3 = 1 << 7
  };

  enum FailStatus2 
  {
    FAIL_STATUS2_OVERCURRENT_DISCHARGE1 = 1 << 0,
    FAIL_STATUS2_OVERCURRENT_CHARGE = 1 << 1,
    FAIL_STATUS2_OVER_TEMPERATURE_CHARGE = 1 << 2,
    FAIL_STATUS2_CELL_UNBALLANCED = 1 << 3,
    FAIL_STATUS2_FULLY_CHARGED = 1 << 4,
    FAIL_STATUS2_DEEP_DISCHARGE = 1 << 5,
    FAIL_STATUS2_OVERCHARGE = 1 << 6,
    FAIL_STATUS2_FET_UNCONTROOL = 1 << 7
  };

  uint8_t fetStatus_{0};
  uint8_t leaderBatteryStatus_{0};
  uint8_t failStatus1_{0};
  uint8_t failStatus2_{0};
};

int main(int argc, char *argv[]) 
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<CanReaderNode>();
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("can_reader_node"), "%s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
