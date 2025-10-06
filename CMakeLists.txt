cmake_minimum_required(VERSION 3.10)
project(hk_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories for Hikvision MVS SDK
# Adjust this path according to where the MVS SDK is installed
set(MVS_SDK_PATH "/opt/MVS" CACHE PATH "Path to MVS SDK")
include_directories(${MVS_SDK_PATH}/include)

# Link directories for Hikvision MVS SDK
link_directories(${MVS_SDK_PATH}/lib/64)

# Add executable
add_executable(hk_camera_node_main src/hk_camera_node_main.cpp src/hk_camera_node.cpp)

# Target include directories
target_include_directories(hk_camera_node_main PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${MVS_SDK_PATH}/include
)

# Target link libraries
target_link_libraries(hk_camera_node_main
  ${OpenCV_LIBS}
  rclcpp::rclcpp
  rclcpp_components::component
  sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
  image_transport::image_transport
  cv_bridge::cv_bridge
  # Hikvision MVS SDK libraries
  MvCameraControl
)

# Install targets
install(TARGETS hk_camera_node_main
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
