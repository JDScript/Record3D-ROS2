cmake_minimum_required(VERSION 3.8)
project(record3d)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 14)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# Add LZFSE compression library
set(LZFSE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs/unix/lzfse-lzfse-1.0)
add_subdirectory(${LZFSE_DIR})
include_directories(${LZFSE_DIR}/src/lzfse)

# Add libusbmuxd for iOS USB communication
set(libusbmuxd_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs/unix/libusbmuxd)
add_subdirectory(${libusbmuxd_DIR})
include_directories(${libusbmuxd_DIR}/include)

# Include Record3D headers
include_directories(include)

# Create Record3D library
add_library(record3d_lib STATIC 
  src/Record3DStream.cpp 
  src/JPEGDecoder.cpp
)
target_link_libraries(record3d_lib PUBLIC lzfse usbmuxd)

# Create ROS node executable
add_executable(record3d_node src/record3d_node.cpp)
target_link_libraries(record3d_node record3d_lib ${OpenCV_LIBS})
ament_target_dependencies(record3d_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Install targets
install(TARGETS
  record3d_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
