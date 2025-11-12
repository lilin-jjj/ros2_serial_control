#ifndef TURBOJET_CONTROL__SIGNAL_GENERATOR_HPP_
#define TURBOJET_CONTROL__SIGNAL_GENERATOR_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "std_msgs/msg/string.hpp"
#include <mutex>
#include <string>


namespace turbojet_control
{

class SignalGenerator : public rclcpp::Node
{
public:
  explicit SignalGenerator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void timer_callback();
  void command_callback(const std_msgs::msg::String::SharedPtr msg);
  
  // Step signal function
  uint16_t step_signal(double t, double time_offset);
  
  // Chirp oscillate function
  uint16_t chirp_oscillate(double t, double time_offset);
  
  // Horizontal transition function
  uint16_t horizontal_transition(double t, double time_offset);
  
  // Ramp signal function
  uint16_t ramp_signal(double t, double time_offset);
  
  // Process chirp helper
  uint16_t process_chirp(double chirp_base, double scale);

  // Command/topic related
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  std::mutex mutex_;
  bool engine_started_ = false;        // auto time-driven sequence enabled
  bool command_override_ = false;      // when true, publish current_throttle_
  uint16_t current_throttle_ = 0;     // throttle set by commands (0-1000)

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr publisher_;
  rclcpp::Time start_time_;
  double time_offset_ = 120.0; // Time offset as in MATLAB code
};

}  // namespace turbojet_control

#endif  // TURBOJET_CONTROL__SIGNAL_GENERATOR_HPP_