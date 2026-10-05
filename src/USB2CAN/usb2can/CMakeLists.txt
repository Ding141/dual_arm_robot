cmake_minimum_required(VERSION 3.16)
project(usb2can)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  msg/CanFdChannelCmd.msg
  msg/DcuCommand.msg
  msg/DcuFeedback.msg
  msg/MotorState.msg
  msg/MotorStateArray.msg
  msg/MotorCommand.msg
  msg/MotorCommandArray.msg
  msg/RawCanFrame.msg
  msg/RawCanFrameArray.msg
  msg/MotorEnable.msg
  msg/MotorEnableArray.msg
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs sensor_msgs
)

include_directories(include)

add_executable(usb2can_node
  src/usb2can_node.cpp
  src/serial_port.cpp
  src/crc16.cpp
  src/protocol.cpp
)

ament_target_dependencies(usb2can_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
)

# Link generated messages (ROS 2 Humble idiomatic)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(usb2can_node ${cpp_typesupport_target} pthread)

add_executable(dm_motor_bridge
  src/dm_motor_bridge.cpp
)
ament_target_dependencies(dm_motor_bridge rclcpp std_msgs)
target_link_libraries(dm_motor_bridge ${cpp_typesupport_target})

install(TARGETS usb2can_node dm_motor_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/test_usb2can.py
  scripts/motor_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
