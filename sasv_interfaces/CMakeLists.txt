cmake_minimum_required(VERSION 3.8)
project(sasv_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(var_dependencies std_msgs vision_msgs geometry_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
foreach(dependency ${var_dependencies})
  find_package(${dependency} REQUIRED)
endforeach()

set(msg_files
  msg/BoundingBox.msg
  msg/ControlCommand.msg
  msg/BoundingBoxArray.msg
  )
rosidl_generate_interfaces(
  ${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES
  ${var_dependencies})
ament_export_dependencies(rosidl_default_runtime ${var_dependencies})

ament_package()