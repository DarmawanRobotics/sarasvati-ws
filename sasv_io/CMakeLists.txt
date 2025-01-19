cmake_minimum_required(VERSION 3.8)
project(sasv_io)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(var_dependencies
    rclcpp
    sensor_msgs
    std_msgs
)

find_package(ament_cmake REQUIRED)

foreach(dependency ${var_dependencies})
  find_package(${dependency} REQUIRED)
endforeach()
find_library(SERIALPORT_LIB serialport REQUIRED)

set(LWGPS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../utils/lwgps")
add_library(lwgps STATIC ${LWGPS_DIR}/src/lwgps.c)
target_include_directories(lwgps PUBLIC ${LWGPS_DIR}/include)

add_executable(f9p_io_node src/f9p_io.cpp)
ament_target_dependencies(f9p_io_node  ${var_dependencies})
target_link_libraries(f9p_io_node lwgps ${SERIALPORT_LIB})


install(TARGETS
  f9p_io_node
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_license_found TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
