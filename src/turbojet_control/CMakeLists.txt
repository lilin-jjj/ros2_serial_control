cmake_minimum_required(VERSION 3.8)
project(turbojet_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(serial_driver REQUIRED)

# Include directories
include_directories(include)

# Create signal generator node
add_executable(signal_generator src/signal_generator.cpp)
ament_target_dependencies(signal_generator rclcpp std_msgs)

# Create serial communication node
add_executable(serial_comm src/serial_comm.cpp)
ament_target_dependencies(serial_comm rclcpp std_msgs serial_driver)

# Install targets
install(TARGETS
  signal_generator
  serial_comm
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  params
  DESTINATION share/${PROJECT_NAME}/
)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()