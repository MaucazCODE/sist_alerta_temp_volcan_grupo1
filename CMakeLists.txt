cmake_minimum_required(VERSION 3.8)
project(sist_alertatemp_volcan)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CustomMsg.msg"
  "srv/CheckSensorStatus.srv"
  "action/CleanSensorMemory.action"
  DEPENDENCIES action_msgs std_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp
)

add_executable(sensorat1 src/sensorat1.cpp)
ament_target_dependencies(sensorat1 rclcpp std_msgs)
target_link_libraries(sensorat1 "${cpp_typesupport_target}")

add_executable(lectorsensor src/lectorsensor.cpp)
ament_target_dependencies(lectorsensor rclcpp std_msgs)
target_link_libraries(lectorsensor "${cpp_typesupport_target}")

add_executable(sensor_cleanup_server src/sensor_cleanup_server.cpp)
ament_target_dependencies(sensor_cleanup_server rclcpp rclcpp_action std_msgs action_msgs)
target_link_libraries(sensor_cleanup_server "${cpp_typesupport_target}")

add_executable(sensor_cleanup_client src/sensor_cleanup_client.cpp)
ament_target_dependencies(sensor_cleanup_client rclcpp rclcpp_action std_msgs action_msgs)
target_link_libraries(sensor_cleanup_client "${cpp_typesupport_target}")

add_executable(central_alert_node src/central_alert_node.cpp)
ament_target_dependencies(central_alert_node rclcpp std_msgs)
target_link_libraries(central_alert_node "${cpp_typesupport_target}")

add_executable(status_service_client src/status_service_client.cpp)
ament_target_dependencies(status_service_client rclcpp std_msgs)
target_link_libraries(status_service_client "${cpp_typesupport_target}")

install(TARGETS
  sensorat1
  lectorsensor
  sensor_cleanup_server
  sensor_cleanup_client
  central_alert_node
  status_service_client
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()