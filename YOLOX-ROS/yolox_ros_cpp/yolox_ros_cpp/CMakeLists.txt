cmake_minimum_required(VERSION 3.5)
project(yolox_ros_cpp)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(bboxes_ex_msgs REQUIRED)
find_package(yolox_cpp)

if(NOT yolox_cpp_FOUND)
  message(WARNING "skipping yolox_ros_cpp, no yolox_cpp found")
  return()
endif()


# executable
add_executable(yolox_ros_cpp
  src/yolox_ros_cpp.cpp
)
ament_target_dependencies(yolox_ros_cpp
  rclcpp
  rclcpp_components
  cv_bridge
  image_transport
  builtin_interfaces
  std_msgs
  sensor_msgs
  OpenCV
  yolox_cpp
  bboxes_ex_msgs
)
target_include_directories(yolox_ros_cpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
install(TARGETS
yolox_ros_cpp
  DESTINATION lib/${PROJECT_NAME}
)

# components
add_library(yolox_ros_cpp_components SHARED
  src/yolox_ros_cpp.cpp
)
rclcpp_components_register_nodes(yolox_ros_cpp_components
 "yolox_ros_cpp::YoloXNode")
target_compile_definitions(yolox_ros_cpp_components
  PRIVATE "YOLOX_ROS_CPP_LIBRARY"
)
target_compile_options(yolox_ros_cpp_components PUBLIC -Wall)

target_include_directories(yolox_ros_cpp_components PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(yolox_ros_cpp_components
  rclcpp
  rclcpp_components
  cv_bridge
  image_transport
  builtin_interfaces
  std_msgs
  sensor_msgs
  OpenCV
  yolox_cpp
  bboxes_ex_msgs
  )

install(TARGETS yolox_ros_cpp_components
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib
)


install(DIRECTORY
  launch
  ../../weights
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
