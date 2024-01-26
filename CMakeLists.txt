cmake_minimum_required(VERSION 3.5)
project(openzen_sensor)

add_compile_options(-std=c++17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)



SET(OPENZEN_TARGET_NAME OpenZen)


set(dependencies
  rclcpp
  std_msgs
  std_srvs
  sensor_msgs
  hardware_interface
  pluginlib
)

## Build 
include_directories(include )

#=== only library for node
add_library(openzen_sensor_lib SHARED src/openzen_sensor_lib.cpp)
target_include_directories(openzen_sensor_lib PRIVATE include)
ament_target_dependencies(openzen_sensor_lib ${dependencies})

target_link_libraries(openzen_sensor_lib
  ${OPENZEN_TARGET_NAME}
)

#=== only library for hw
add_library(openzen_sensor_lib_hw SHARED src/imu_sensor_lib_hw.cpp)
target_include_directories(openzen_sensor_lib_hw PRIVATE include)
ament_target_dependencies(openzen_sensor_lib_hw ${dependencies})

target_link_libraries(openzen_sensor_lib_hw
  ${OPENZEN_TARGET_NAME}
)

#=== Executable node
add_executable(openzen_sensor_node
  src/OpenZenNode.cpp
)

ament_target_dependencies(openzen_sensor_node ${dependencies})

target_link_libraries(openzen_sensor_node
  ${OPENZEN_TARGET_NAME}
  openzen_sensor_lib
)

#=== Hardware interface
add_library(imu_sensor_hardware SHARED src/imu_sensor_hardware.cpp)
target_include_directories(imu_sensor_hardware PRIVATE include)
ament_target_dependencies(imu_sensor_hardware ${dependencies})
target_link_libraries(imu_sensor_hardware openzen_sensor_lib_hw)

#=== Export hardware plugin
pluginlib_export_plugin_description_file(hardware_interface imu_sensor_hardware.xml)

install(TARGETS openzen_sensor_lib openzen_sensor_lib_hw imu_sensor_hardware
  DESTINATION lib

)

## install node binary
install(TARGETS openzen_sensor_node 
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch urdf config
  DESTINATION share/${PROJECT_NAME}
)


ament_package()