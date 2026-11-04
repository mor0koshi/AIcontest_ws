cmake_minimum_required(VERSION 3.8)
project(aicontest_can_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/can_bridge.cpp
  src/serial_bridge.cpp
)

# CAN 版(Pi も CAN バスにつなぐとき)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "can_bridge::can_bridge"
  EXECUTABLE can_bridge_node
)
# UART 版(Nucleo の USB でつなぐ。既定)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "serial_bridge::serial_bridge"
  EXECUTABLE serial_bridge_node
)

install(PROGRAMS scripts/fake_nucleo.py scripts/fake_nucleo_serial.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_protocol test/test_protocol.cpp)
  target_include_directories(test_protocol PRIVATE include)
  ament_add_gtest(test_serial_framing test/test_serial_framing.cpp)
  target_include_directories(test_serial_framing PRIVATE include)
endif()

ament_auto_package(INSTALL_TO_SHARE config launch)
