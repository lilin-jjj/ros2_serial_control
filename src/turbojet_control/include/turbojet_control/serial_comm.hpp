#ifndef TURBOJET_CONTROL__SERIAL_COMM_HPP_
#define TURBOJET_CONTROL__SERIAL_COMM_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16.hpp"
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
  uint8_t compute_crc8_reflected(const std::vector<uint8_t>& data_bytes);
  bool open_serial_port();
  void close_serial_port();
  bool send_data(const std::vector<uint8_t>& data);
  void receive_feedback_thread();  // 接收反馈的线程
  void parse_ecu_feedback(const std::vector<uint8_t>& data);  // 解析 ECU 反馈
  
  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr subscription_;
  std::shared_ptr<drivers::common::IoContext> io_context_;
  std::unique_ptr<drivers::serial_driver::SerialPort> serial_port_;
  std::thread feedback_thread_;  // 反馈接收线程
  std::mutex feedback_mutex_;
  volatile bool running_ = true;  // 控制反馈线程循环
  
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