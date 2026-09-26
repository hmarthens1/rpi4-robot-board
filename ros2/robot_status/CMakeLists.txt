cmake_minimum_required(VERSION 3.16)
project(robot_status LANGUAGES C CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(nlohmann_json 3 REQUIRED)            # apt: nlohmann-json3-dev
find_package(PkgConfig REQUIRED)
pkg_check_modules(GPIODCXX REQUIRED libgpiodcxx)  # apt: libgpiod-dev (1.6)

# ---- rpi_ws281x (the WS2812 RGB LEDs). Not packaged for Ubuntu: fetch the C
# sources at a pinned commit (has Pi 4 rev c03115) and compile them into this
# package. Its own CMakeLists would install system-wide and run ldconfig, so
# only the sources are used.
include(FetchContent)
FetchContent_Declare(rpi_ws281x
  GIT_REPOSITORY https://github.com/jgarff/rpi_ws281x.git
  GIT_TAG 22f2e35012704845d82e06d2fa2e10fbc810ecaa)
FetchContent_GetProperties(rpi_ws281x)
if(NOT rpi_ws281x_POPULATED)
  FetchContent_Populate(rpi_ws281x)
endif()
set(WS_DIR ${rpi_ws281x_SOURCE_DIR})
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/ws281x/version.h
  "#pragma once\n#define VERSION_MAJOR 1\n#define VERSION_MINOR 1\n#define VERSION_MICRO 0\n")
add_library(ws2811 STATIC
  ${WS_DIR}/mailbox.c ${WS_DIR}/ws2811.c ${WS_DIR}/pwm.c
  ${WS_DIR}/pcm.c ${WS_DIR}/dma.c ${WS_DIR}/rpihw.c)
target_include_directories(ws2811 PUBLIC ${WS_DIR} ${CMAKE_CURRENT_BINARY_DIR}/ws281x)
target_compile_options(ws2811 PRIVATE -w)          # third-party code: don't flood the build log
set_target_properties(ws2811 PROPERTIES POSITION_INDEPENDENT_CODE ON C_STANDARD 11)
target_link_libraries(ws2811 m)

# ---- the board driver (I2C: motors, servos, battery, sonar) - no ROS in here
add_library(robot_board STATIC src/i2c.cpp src/board.cpp src/sonar.cpp)
target_include_directories(robot_board PUBLIC include)
set_target_properties(robot_board PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(robot_board_gpio STATIC src/peripherals.cpp)
target_include_directories(robot_board_gpio PUBLIC include ${GPIODCXX_INCLUDE_DIRS})
target_link_libraries(robot_board_gpio ws2811 ${GPIODCXX_LIBRARIES})

# ---- the two nodes
add_executable(status_node src/status_node.cpp)
target_link_libraries(status_node robot_board)
ament_target_dependencies(status_node rclcpp sensor_msgs diagnostic_msgs)

add_executable(command_node src/command_node.cpp)
target_link_libraries(command_node robot_board robot_board_gpio nlohmann_json::nlohmann_json)
ament_target_dependencies(command_node rclcpp std_msgs geometry_msgs rcl_interfaces)

install(TARGETS status_node command_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_board test/test_board.cpp)
  target_link_libraries(test_board robot_board)
endif()

ament_package()
