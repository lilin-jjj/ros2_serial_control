#include "turbojet_control/signal_generator.hpp"
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <sstream>
#include <algorithm>
#include <cctype>

using namespace std::chrono_literals;

namespace turbojet_control
{

SignalGenerator::SignalGenerator(const rclcpp::NodeOptions & options)
: Node("signal_generator", options)
{
  // Create publisher for throttle signal
  publisher_ = this->create_publisher<std_msgs::msg::UInt16>("throttle_signal", 10);

  // Create command subscriber
  command_sub_ = this->create_subscription<std_msgs::msg::String>(
    "control_command", 10, std::bind(&SignalGenerator::command_callback, this, std::placeholders::_1));

  // Create timer to generate signal at 20Hz (0.05s as in MATLAB)
  timer_ = this->create_wall_timer(
    50ms, std::bind(&SignalGenerator::timer_callback, this));

  // Record start time (can be reset by 'start' command)
  start_time_ = this->now();

  RCLCPP_INFO(this->get_logger(), "Signal Generator node started (topic: throttle_signal). Listening for control_command.");
}

void SignalGenerator::timer_callback()
{
  // Decide publish value based on command override or auto sequence
  uint16_t throttle_value = 0;

  {
    std::lock_guard<std::mutex> lk(mutex_);
    
    // Debug: Print state every second
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "State: override=%d, started=%d, current_throttle=%d",
                         command_override_, engine_started_, current_throttle_);
    
    if (command_override_) {
      // command-driven value
      throttle_value = current_throttle_;
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Mode: COMMAND_OVERRIDE, throttle=%d", throttle_value);
    } else if (!engine_started_) {
      // engine not started and no override -> publish 0
      throttle_value = 0;
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Mode: ENGINE_NOT_STARTED, throttle=0");
    } else {
      // Auto/time-driven sequence (use original code)
      auto current_time = this->now();
      double elapsed = (current_time - start_time_).nanoseconds() / 1e9;

      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Mode: AUTO, elapsed=%.2fs, offset=%.2fs", elapsed, time_offset_);

      if (elapsed < 300 + time_offset_) {
        throttle_value = step_signal(elapsed, time_offset_);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Signal: step_signal(%.2f, %.2f) = %d", elapsed, time_offset_, throttle_value);
      } else if (elapsed >= 300 + time_offset_ && elapsed < 400 + time_offset_) {
        throttle_value = chirp_oscillate(elapsed, time_offset_);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Signal: chirp_oscillate = %d", throttle_value);
      } else if (elapsed >= 400 + time_offset_ && elapsed < 405 + time_offset_) {
        throttle_value = horizontal_transition(elapsed, time_offset_);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Signal: horizontal_transition = %d", throttle_value);
      } else if (elapsed >= 405 + time_offset_ && elapsed < 505 + time_offset_) {
        throttle_value = ramp_signal(elapsed, time_offset_);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Signal: ramp_signal = %d", throttle_value);
      } else {
        throttle_value = 700; // Constant value after 555s
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Signal: constant = 700");
      }
    }
  }

  // Ensure value is within bounds
  throttle_value = std::max(0, std::min(1000, static_cast<int>(throttle_value)));

  // Publish the signal
  auto message = std_msgs::msg::UInt16();
  message.data = throttle_value;
  publisher_->publish(message);

  RCLCPP_DEBUG(this->get_logger(), "Publishing throttle signal: %d", throttle_value);
}

void SignalGenerator::command_callback(const std_msgs::msg::String::SharedPtr msg)
{
  if (!msg) return;

  // Convert to lower-case and tokenize
  std::string s = msg->data;
  // trim
  s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](unsigned char ch) { return !std::isspace(ch); }));
  s.erase(std::find_if(s.rbegin(), s.rend(), [](unsigned char ch) { return !std::isspace(ch); }).base(), s.end());
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c){ return std::tolower(c); });

  std::istringstream iss(s);
  std::string cmd, arg;
  iss >> cmd;
  iss >> arg; // optional

  std::lock_guard<std::mutex> lk(mutex_);

  if (cmd == "start") {
    engine_started_ = true;
    command_override_ = false; // go into auto/time-driven mode
    start_time_ = this->now();
    RCLCPP_INFO(this->get_logger(), "Received 'start' command: entering auto time-driven sequence");
  } else if (cmd == "stop") {
    engine_started_ = false;
    command_override_ = true;
    current_throttle_ = 0;
    RCLCPP_INFO(this->get_logger(), "Received 'stop' command: engine stopped, throttle=0");
  } else if (cmd == "auto" || cmd == "resume") {
    // resume automatic/time-based sequence without resetting time
    command_override_ = false;
    engine_started_ = true;
    RCLCPP_INFO(this->get_logger(), "Received 'auto'/'resume' command: resuming auto sequence");
  } else if (cmd == "oill" || cmd == "oil") {
    // support 'oill add' and 'oill sub' or 'oil add'/'oil sub'
    if (arg == "add" || arg == "+") {
      // increase by 5% of full scale (1000) -> 50 units
      uint16_t delta = static_cast<uint16_t>(std::round(0.05 * 1000.0));
      uint32_t tmp = static_cast<uint32_t>(current_throttle_) + delta;
      current_throttle_ = static_cast<uint16_t>(std::min<uint32_t>(1000, tmp));
      command_override_ = true;
      RCLCPP_INFO(this->get_logger(), "Received '%s %s' command: throttle -> %u", cmd.c_str(), arg.c_str(), current_throttle_);
    } else if (arg == "sub" || arg == "-") {
      uint16_t delta = static_cast<uint16_t>(std::round(0.05 * 1000.0));
      int tmp = static_cast<int>(current_throttle_) - static_cast<int>(delta);
      current_throttle_ = static_cast<uint16_t>(std::max(0, tmp));
      command_override_ = true;
      RCLCPP_INFO(this->get_logger(), "Received '%s %s' command: throttle -> %u", cmd.c_str(), arg.c_str(), current_throttle_);
    } else {
      RCLCPP_WARN(this->get_logger(), "Unknown '%s' subcommand: '%s'", cmd.c_str(), arg.c_str());
    }
  } else if (cmd == "set") {
    // set <value>
    if (!arg.empty()) {
      try {
        int val = std::stoi(arg);
        val = std::max(0, std::min(1000, val));
        current_throttle_ = static_cast<uint16_t>(val);
        command_override_ = true;
        RCLCPP_INFO(this->get_logger(), "Received 'set' command: throttle set to %u", current_throttle_);
      } catch (const std::exception & e) {
        RCLCPP_WARN(this->get_logger(), "Invalid value for set command: '%s'", arg.c_str());
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "'set' command requires a numeric argument");
    }
  } else {
    RCLCPP_WARN(this->get_logger(), "Unknown control command: '%s'", s.c_str());
  }
}

uint16_t SignalGenerator::step_signal(double t, double time_offset)
{
  double val = 0;
  
  if (t < 15 + time_offset) {
    val = 0;           // Initial value
  } else if (t >= 15 + time_offset && t < 30 + time_offset) {
    val = 200;         // +200
  } else if (t >= 30 + time_offset && t < 45 + time_offset) {
    val = 0;           // -200
  } else if (t >= 45 + time_offset && t < 60 + time_offset) {
    val = 400;         // +400
  } else if (t >= 60 + time_offset && t < 75 + time_offset) {
    val = 0;           // -400
  } else if (t >= 75 + time_offset && t < 90 + time_offset) {
    val = 600;         // +600
  } else if (t >= 90 + time_offset && t < 105 + time_offset) {
    val = 0;           // -600
  } else if (t >= 105 + time_offset && t < 120 + time_offset) {
    val = 800;         // +800
  } else if (t >= 120 + time_offset && t < 135 + time_offset) {
    val = 0;           // -800
  } else if (t >= 135 + time_offset && t < 150 + time_offset) {
    val = 1000;        // +1000
  } else if (t >= 150 + time_offset && t < 165 + time_offset) {
    val = 500;         // -500
  } else if (t >= 165 + time_offset && t < 180 + time_offset) {
    val = 300;         // -200
  } else if (t >= 180 + time_offset && t < 195 + time_offset) {
    val = 800;         // +500
  } else if (t >= 195 + time_offset && t < 210 + time_offset) {
    val = 400;         // -400
  } else if (t >= 210 + time_offset && t < 225 + time_offset) {
    val = 950;         // +550
  } else if (t >= 225 + time_offset && t < 240 + time_offset) {
    val = 150;         // -800
  } else if (t >= 240 + time_offset && t < 255 + time_offset) {
    val = 600;         // +450
  } else if (t >= 255 + time_offset && t < 270 + time_offset) {
    val = 350;         // -250
  } else if (t >= 270 + time_offset && t < 285 + time_offset) {
    val = 320;         // -30
  } else {
    val = 600;         // Final value
  }
  
  // Clamp value between 0 and 1000
  val = std::max(0.0, std::min(1000.0, val));
  return static_cast<uint16_t>(std::round(val));
}

uint16_t SignalGenerator::process_chirp(double chirp_base, double scale)
{
  double scaled = (chirp_base + 1.0) / 2.0 * scale;
  double val = std::round(scaled);
  val = std::max(0.0, std::min(1000.0, val));
  return static_cast<uint16_t>(val);
}

double chirp_impl(double t, double f0, double t1, double f1, const std::string& method)
{
  // Simplified logarithmic chirp implementation
  // In a real implementation, you would use a more accurate chirp function
  double B = f1 - f0;
  double total_time = t1; // Assuming t0 = 0
  
  if (method == "logarithmic") {
    // Simplified log chirp: f(t) = f0 * (f1/f0)^(t/t1)
    // Phase = 2*pi * integral(f(t) dt) = 2*pi * f0 * t1 / ln(f1/f0) * [(f1/f0)^(t/t1) - 1]
    if (f0 > 0 && f1 > 0 && total_time > 0) {
      double ratio = f1 / f0;
      if (ratio != 1.0) {
        double beta = total_time / std::log(ratio);
        double phase = 2 * M_PI * beta * f0 * (std::pow(ratio, t/total_time) - 1.0);
        return std::sin(phase);
      }
    }
  }
  
  // Default to linear chirp if logarithmic fails or is not specified
  double k = B / total_time; // Rate of frequency change
  double phase = 2 * M_PI * (f0 * t + 0.5 * k * t * t);
  return std::sin(phase);
}

uint16_t SignalGenerator::chirp_oscillate(double t, double time_offset)
{
  uint16_t val = 0;
  
  if (t >= 300 + time_offset && t < 345 + time_offset) {
    double chirp_base = chirp_impl(t, 0.000001 * M_PI, 350 + time_offset, 0.2 * M_PI, "logarithmic");
    val = process_chirp(chirp_base, 500) + 450;
  } else if (t >= 345 + time_offset && t < 350 + time_offset) {
    val = 450;
  } else { // 350 <= t < 400
    double chirp_base = chirp_impl(t, 0.000001 * M_PI, 400 + time_offset, 0.2 * M_PI, "logarithmic");
    val = process_chirp(chirp_base, 600) - 50;
  }
  
  return val;
}

uint16_t SignalGenerator::horizontal_transition(double t, double time_offset)
{
  // Constant value of 300
  return 300;
}

uint16_t SignalGenerator::ramp_signal(double t, double time_offset)
{
  double start_t, end_t, start_val, end_val;
  
  if (t >= 405 + time_offset && t < 435 + time_offset) {
    start_t = 405 + time_offset;
    end_t = 435 + time_offset;
    start_val = 300;
    end_val = 1000;
  } else if (t >= 435 + time_offset && t < 475 + time_offset) {
    start_t = 435 + time_offset;
    end_t = 475 + time_offset;
    start_val = 1000;
    end_val = 0;
  } else { // 475 <= t < 505
    start_t = 475 + time_offset;
    end_t = 505 + time_offset;
    start_val = 0;
    end_val = 700;
  }
  
  double slope = (end_val - start_val) / (end_t - start_t);
  double val = start_val + slope * (t - start_t);
  val = std::round(val);
  val = std::max(0.0, std::min(1000.0, val));
  
  return static_cast<uint16_t>(val);
}

}  // namespace turbojet_control

// Main function
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<turbojet_control::SignalGenerator>());
  rclcpp::shutdown();
  return 0;
}