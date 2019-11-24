cmake_minimum_required(VERSION 3.5)
project(optris_drivers2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)


include_directories(
  /usr/include/libirimager/
  include
)

set(msg_files
  "msg/Temperature.msg"
  "msg/Flag.msg"
)

set(srv_files
  "srv/AutoFlag.srv"
  "srv/TemperatureRange.srv"
  "srv/Palette.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces std_msgs
)
ament_export_dependencies(rosidl_default_runtime)

add_executable(optris_imager_node src/optris_imager_node.cpp src/OptrisImager.cpp)
ament_target_dependencies(optris_imager_node rclcpp std_msgs std_srvs sensor_msgs image_transport camera_info_manager)
rosidl_target_interfaces(optris_imager_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(optris_colorconvert_node src/optris_colorconvert_node.cpp src/OptrisColorconvert.cpp)
ament_target_dependencies(optris_colorconvert_node rclcpp std_msgs std_srvs sensor_msgs image_transport camera_info_manager)
rosidl_target_interfaces(optris_colorconvert_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(optris_imager_node 
  irimager
  irimageprocessing
  ircore
  udev
  usb-1.0)

target_link_libraries(optris_colorconvert_node 
  irimager
  irimageprocessing
  ircore
  udev
  usb-1.0)

install(TARGETS
  optris_imager_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  optris_colorconvert_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
