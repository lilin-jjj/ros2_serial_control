#include "turbojet_control/serial_comm.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <iostream>
#include <bitset>
#include <thread>

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
  
  // Create subscriber for control commands
  command_subscription_ = this->create_subscription<std_msgs::msg::String>(
    "control_command", 10,
    std::bind(&SerialComm::command_callback, this, std::placeholders::_1));
  
  // Initialize engine start time
  engine_start_time_ = this->now();
  
  // Start feedback receiving thread
  feedback_thread_ = std::thread(&SerialComm::receive_feedback_thread, this);
  
  RCLCPP_INFO(this->get_logger(), "SerialComm node started, listening on throttle_signal and control_command topics");
  RCLCPP_INFO(this->get_logger(), "ECU feedback receiver thread started");
}

void SerialComm::throttle_callback(const std_msgs::msg::UInt16::SharedPtr msg)
{
  // Get SW state based on engine running time
  uint8_t SW = get_sw_state();
  
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

bool SerialComm::send_data(const std::vector<uint8_t>& data)
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

void SerialComm::receive_feedback_thread()
{
  RCLCPP_INFO(this->get_logger(), "ECU feedback receiver thread running");
  
  std::vector<uint8_t> buffer;
  const size_t MAX_BUFFER = 1024;
  
  while (running_) {
    if (!serial_port_ || !serial_port_->is_open()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }
    
    try {
      // Try to receive data from serial port
      std::vector<uint8_t> data;
      if (serial_port_->receive(data)) {
        buffer.insert(buffer.end(), data.begin(), data.end());
        
        // Try to parse complete messages from buffer
        // Assume ECU feedback format: [Header(1B)][Data(variable)][CRC(1B)]
        // Common ECU response: 0xFF + data_bytes + CRC
        while (buffer.size() >= 4) {  // Minimum: header + 2 data bytes + CRC
          if (buffer[0] == FRAME_HEAD) {
            // Try to find a complete message
            // For now, assume 4-byte response (can be extended)
            if (buffer.size() >= 4) {
              std::vector<uint8_t> msg(buffer.begin(), buffer.begin() + 4);
              parse_ecu_feedback(msg);
              buffer.erase(buffer.begin(), buffer.begin() + 4);
            } else {
              break;
            }
          } else {
            // Discard invalid byte and search for next frame header
            buffer.erase(buffer.begin());
          }
        }
        
        // Prevent buffer overflow
        if (buffer.size() > MAX_BUFFER) {
          RCLCPP_WARN(this->get_logger(), "Feedback buffer overflow, clearing");
          buffer.clear();
        }
      }
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "Error reading from serial port: %s", e.what());
    }
    
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  
  RCLCPP_INFO(this->get_logger(), "ECU feedback receiver thread stopped");
}

void SerialComm::parse_ecu_feedback(const std::vector<uint8_t>& data)
{
  if (data.size() < 4) {
    RCLCPP_WARN(this->get_logger(), "Invalid feedback message size: %zu", data.size());
    return;
  }
  
  uint8_t header = data[0];
  uint8_t byte1 = data[1];
  uint8_t byte2 = data[2];
  uint8_t crc = data[3];
  
  // Verify frame header
  if (header != FRAME_HEAD) {
    RCLCPP_WARN(this->get_logger(), "Invalid frame header: 0x%02X", header);
    return;
  }
  
  // Verify CRC
  std::vector<uint8_t> crc_data = {byte1, byte2};
  uint8_t computed_crc = compute_crc8_reflected(crc_data);
  if (computed_crc != crc) {
    RCLCPP_WARN(this->get_logger(), 
                "CRC mismatch: received=0x%02X, computed=0x%02X", crc, computed_crc);
    return;
  }
  
  // Parse ECU feedback based on byte1 and byte2
  // Typical ECU feedback format (customize based on your ECU protocol):
  // Byte1: Status flags (engine status, error codes, etc.)
  // Byte2: Engine parameters (temperature, RPM, etc.)
  
  uint8_t engine_status = (byte1 >> 4) & 0x0F;  // High 4 bits
  uint8_t error_code = byte1 & 0x0F;             // Low 4 bits
  uint8_t temperature = byte2;                   // Temperature (0-255, scaled)
  
  // Log the parsed information
  RCLCPP_INFO(this->get_logger(),
              "ECU Feedback [%02X %02X %02X %02X]: "
              "Status=0x%X, Error=0x%X, Temp=%u (raw)",
              header, byte1, byte2, crc,
              engine_status, error_code, temperature);
  
  // Parse engine status
  std::string status_str;
  switch (engine_status) {
    case 0x0: status_str = "Off"; break;
    case 0x1: status_str = "Starting"; break;
    case 0x2: status_str = "Idle"; break;
    case 0x3: status_str = "Running"; break;
    case 0x4: status_str = "Stopping"; break;
    case 0x5: status_str = "Fault"; break;
    default: status_str = "Unknown"; break;
  }
  
  // Parse error code
  std::string error_str;
  switch (error_code) {
    case 0x0: error_str = "No Error"; break;
    case 0x1: error_str = "Over Temperature"; break;
    case 0x2: error_str = "Low Voltage"; break;
    case 0x3: error_str = "High Voltage"; break;
    case 0x4: error_str = "Sensor Error"; break;
    case 0x5: error_str = "Fuel Error"; break;
    case 0xF: error_str = "Unspecified Error"; break;
    default: error_str = "Reserved Error"; break;
  }
  
  // Convert temperature (example: raw value * 0.5 - 40°C)
  float temp_celsius = temperature * 0.5f - 40.0f;
  
  RCLCPP_INFO(this->get_logger(),
              "[ECU Status] Engine: %s | Error: %s | Temperature: %.1f°C",
              status_str.c_str(), error_str.c_str(), temp_celsius);
}

void SerialComm::command_callback(const std_msgs::msg::String::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  
  std::string command = msg->data;
  
  if (command == "start") {
    engine_started_ = true;
    engine_start_time_ = this->now();
    RCLCPP_INFO(this->get_logger(), "Engine started - SW state will follow time sequence");
  } else if (command == "stop") {
    engine_started_ = false;
    RCLCPP_INFO(this->get_logger(), "Engine stopped - SW state set to STOP (1)");
  } else if (command == "auto" || command == "resume") {
    // Resume automatic mode without resetting time
    engine_started_ = true;
    RCLCPP_INFO(this->get_logger(), "Resumed automatic mode");
  }
  // Note: For set/add/sub commands, we don't change engine state
  // The engine state is managed by start/stop commands
}

uint8_t SerialComm::get_sw_state()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  
  if (!engine_started_) {
    return 1;  // Stop state - 停止状态（超温不冷却）
  }
  
  // Calculate elapsed time since engine start
  auto current_time = this->now();
  double elapsed = (current_time - engine_start_time_).seconds();
  
  // Calculate SW state based on engine running time
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
  
  return SW;
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
