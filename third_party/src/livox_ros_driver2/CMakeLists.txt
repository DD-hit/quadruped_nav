cmake_minimum_required(VERSION 3.8)
project(livox_ros_driver2)
# Message-only extraction: no driver executable, SDK, network or hardware setup.
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CustomPoint.msg" "msg/CustomMsg.msg" DEPENDENCIES std_msgs)
ament_export_dependencies(rosidl_default_runtime)
install(FILES README.md UPSTREAM_LICENSE DESTINATION share/${PROJECT_NAME})
ament_package()
