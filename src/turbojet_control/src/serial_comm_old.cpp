#include "turbojet_control/serial_comm.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <iostream>
#include <bitset>

// Linux serial headers
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>

using namespace std::chrono_literals;

namespace turbojet_control
{

SerialComm::SerialComm(const rclcpp::NodeOptions & options)
: Node("serial_comm", options), io_context_(std::make_shared<drivers::common::IoContext>(2)), serial_port_(nullptr)
{
  // Declare and get parameters
  port_name_ = this->declare_parameter("port_name", "/dev/ttyUSB0");
  baud_rate_ = this->declare_parameter("baud_rate", 9600);
  
  // Initialize serial port
  if (!open_serial_port()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open serial port %s", port_name_.c_str());
  } else {
    RCLCPP_INFO(this->get_logger(), "Serial port %s opened successfully", port_name_.c_str());
  }
  
  // Create subscriber for throttle signal
  subscription_ = this->create_subscription<std_msgs::msg::UInt16>(
    "throttle_signal", 10, 
    std::bind(&SerialComm::throttle_callback, this, std::placeholders::_1));
  
  // Start feedback receiving thread
  feedback_thread_ = std::thread(&SerialComm::receive_feedback_thread, this);
  
  RCLCPP_INFO(this->get_logger(), "SerialComm node started, listening on throttle_signal topic");
  RCLCPP_INFO(this->get_logger(), "ECU feedback receiver thread started");
}

void SerialComm::throttle_callback(const std_msgs::msg::UInt16::SharedPtr msg)
{
  // Get current time for SW state calculation
  auto current_time = this->now();
  double elapsed = current_time.nanoseconds() / 1e9; // Convert to seconds
  
  // Calculate SW state based on time (similar to MATLAB logic)
  uint8_t SW;
  if (elapsed < 1.0) {
    SW = 0;        // Serial not controlled - 不控制引擎（PWM输入控制模式）
  } else if (elapsed >= 1.0 && elapsed < 3.0) {
    SW = 2;        // Decimal 2 (binary 10) - 待机状态（超温自动冷却）
  } else if (elapsed >= 3.0 && elapsed < 560.0) {
    SW = 3;        // Normal operation (binary 11) - 运行状态
  } else {
    SW = 1;        // Backup state - 停止状态（超温不冷却）
  }
  
  // Limit throttle value to 0-1000 range
  uint16_t throttle_limited = std::max(0, std::min(1000, static_cast<int>(msg->data)));
  
  // Convert throttle value to 10-bit binary representation
  // Split into high 2 bits and low 8 bits
  uint8_t throttle_high2 = (throttle_limited >> 8) & 0x03;  // High 2 bits
  uint8_t throttle_low8 = throttle_limited & 0xFF;          // Low 8 bits
  
  // Construct message bytes according to ECU protocol
  uint8_t byte0 = FRAME_HEAD;  // 0xFF
  uint8_t byte1 = (CMD_ID << 4) | (SW << 2) | throttle_high2;
  uint8_t byte2 = throttle_low8;
  
  // Calculate CRC for bytes 1 and 2
  std::vector<uint8_t> data_bytes = {byte1, byte2};
  uint8_t byte3 = compute_crc8_reflected(data_bytes);
  
  // Create the complete 4-byte message
  std::vector<uint8_t> ecu_message = {byte0, byte1, byte2, byte3};
  
  // Send message via serial port
  if (send_data(ecu_message)) {
    RCLCPP_DEBUG(this->get_logger(), 
                 "Sent ECU message: [%02X %02X %02X %02X]", 
                 byte0, byte1, byte2, byte3);
  } else {
    RCLCPP_WARN(this->get_logger(), "Failed to send ECU message");
    // Print message for debugging
    RCLCPP_DEBUG(this->get_logger(), 
                 "ECU message: [%02X %02X %02X %02X]", 
                 byte0, byte1, byte2, byte3);
  }
}

uint8_t SerialComm::compute_crc8_reflected(const std::vector<uint8_t>& data_bytes)
{
  // CRC parameters according to ECU protocol (reflected CRC with poly 0x8C)
  uint8_t crc = 0;      // Initial value
  
  for (size_t i = 0; i < data_bytes.size(); i++) {
    uint8_t b = data_bytes[i];
    crc ^= b;  // crc ^= byte
    
    // Process 8 bits of the current crc (LSB first for reflected CRC)
    for (int k = 0; k < 8; k++) {
      if (crc & 1) {
        // If LSB is 1, shift right and XOR with polynomial
        crc = (crc >> 1) ^ 0x8C;
      } else {
        // Otherwise just shift right
        crc = crc >> 1;
      }
    }
  }
  
  return crc;
}

bool SerialComm::open_serial_port()
{
  // Close if already open
  if (serial_port_ && serial_port_->is_open()) {
    serial_port_->close();
  }
  
  try {
    // Create serial port configuration
    drivers::serial_driver::SerialPortConfig config(
      baud_rate_,
      drivers::serial_driver::FlowControl::NONE,
      drivers::serial_driver::Parity::NONE,
      drivers::serial_driver::StopBits::ONE
    );
    
    // Create serial port
    serial_port_ = std::make_unique<drivers::serial_driver::SerialPort>(
      *io_context_,
      port_name_,
      config
    );
    
    // Open serial port
    serial_port_->open();
    
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error opening serial port: %s", e.what());
    return false;
  }
}

void SerialComm::close_serial_port()
{
  if (serial_port_ && serial_port_->is_open()) {
    serial_port_->close();
  }
}
{
  if (!serial_port_ || !serial_port_->is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Serial port not open");
    return false;
  }
  
  try {
    serial_port_->send(data);
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error writing to serial port: %s", e.what());
    return false;
  }
}

}  // namespace turbojet_control

// Main function
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<turbojet_control::SerialComm>());
  rclcpp::shutdown();
  return 0;
}