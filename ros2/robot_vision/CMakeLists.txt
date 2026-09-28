cmake_minimum_required(VERSION 3.16)
project(robot_vision LANGUAGES CXX)

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
find_package(geometry_msgs REQUIRED)
find_package(nlohmann_json 3 REQUIRED)                                   # apt: nlohmann-json3-dev
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc imgcodecs videoio)   # apt: libopencv-dev
find_package(ncnn REQUIRED)                  # built by scripts/install_vision.sh into /usr/local

# ---- vision library (no ROS): lane tracking and YOLOX
add_library(robot_vision_core STATIC src/lane.cpp src/lane_seeker.cpp src/yolox.cpp)
target_include_directories(robot_vision_core PUBLIC include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(robot_vision_core ${OpenCV_LIBS} ncnn)
set_target_properties(robot_vision_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(vision_node src/vision_node.cpp)
target_link_libraries(vision_node robot_vision_core nlohmann_json::nlohmann_json)
ament_target_dependencies(vision_node rclcpp std_msgs sensor_msgs geometry_msgs)

add_executable(vision_bench src/vision_bench.cpp)
target_link_libraries(vision_bench robot_vision_core)

install(TARGETS vision_node vision_bench DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_lane test/test_lane.cpp)
  target_link_libraries(test_lane robot_vision_core)
  target_compile_definitions(test_lane PRIVATE TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data")
  ament_add_gtest(test_seeker test/test_seeker.cpp)
  target_link_libraries(test_seeker robot_vision_core)
endif()

ament_package()
