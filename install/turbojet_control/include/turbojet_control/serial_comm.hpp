#ifndef TURBOJET_CONTROL__SERIAL_COMM_HPP_
#define TURBOJET_CONTROL__SERIAL_COMM_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "std_msgs/msg/string.hpp"
#include "serial_driver/serial_port.hpp"
#include "serial_driver/serial_driver.hpp"

#include <string>
#include <vector>
#include <memory>
#include <thread>
#include <mutex>

namespace turbojet_control
{

class SerialComm : public rclcpp::Node
{
public:
  explicit SerialComm(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void throttle_callback(const std_msgs::msg::UInt16::SharedPtr msg);
  void command_callback(const std_msgs::msg::String::SharedPtr msg);  // 命令回调
  uint8_t compute_crc8_reflected(const std::vector<uint8_t>& data_bytes);
  bool open_serial_port();
  void close_serial_port();
  bool send_data(const std::vector<uint8_t>& data);
  void receive_feedback_thread();  // 接收反馈的线程
  void parse_ecu_feedback(const std::vector<uint8_t>& data);  // 解析 ECU 反馈
  uint8_t get_sw_state();  // 获取当前 SW 状态
  
  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_subscription_;  // 命令订阅
  std::shared_ptr<drivers::common::IoContext> io_context_;
  std::unique_ptr<drivers::serial_driver::SerialPort> serial_port_;
  std::thread feedback_thread_;  // 反馈接收线程
  std::mutex feedback_mutex_;
  std::mutex state_mutex_;  // 保护引擎状态
  volatile bool running_ = true;  // 控制反馈线程循环
  
  // 引擎状态管理
  bool engine_started_ = false;  // 引擎是否启动
  rclcpp::Time engine_start_time_;  // 引擎启动时间
  
  // ECU protocol parameters
  const uint8_t FRAME_HEAD = 0xFF;
  const uint8_t CMD_ID = 1;
  
  // Serial port parameters
  std::string port_name_;
  int baud_rate_;
  int time_offset_ = 120; // Time offset as in MATLAB code
};

}  // namespace turbojet_control

#endif  // TURBOJET_CONTROL__SERIAL_COMM_HPP_