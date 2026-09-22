cmake_minimum_required(VERSION 3.8)
project(robocolumbus_stuff)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robocolumbus_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(tof_node src/tof_node.cpp)
ament_target_dependencies(tof_node
  rclcpp
  robocolumbus_interfaces
  std_msgs
)
target_link_libraries(tof_node nlohmann_json::nlohmann_json)

add_executable(wheel_controler_node src/wheel_controler_node.cpp)
ament_target_dependencies(wheel_controler_node
  geometry_msgs
  nav_msgs
  rclcpp
  std_msgs
)
target_link_libraries(wheel_controler_node nlohmann_json::nlohmann_json)

add_executable(imu_gps_node src/imu_gps_node.cpp)
ament_target_dependencies(imu_gps_node
  rclcpp
  robocolumbus_interfaces
  sensor_msgs
  std_msgs
)
target_link_libraries(imu_gps_node nlohmann_json::nlohmann_json)

install(TARGETS tof_node wheel_controler_node imu_gps_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  robocolumbus_stuff/cone_node.py
  robocolumbus_stuff/controller_node.py
  robocolumbus_stuff/nav_node.py
  robocolumbus_stuff/pcd_node.py
  robocolumbus_stuff/speaker_node.py
  robocolumbus_stuff/teleop_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch resource
  DESTINATION share/${PROJECT_NAME}
)

ament_package()