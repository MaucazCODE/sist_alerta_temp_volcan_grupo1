cmake_minimum_required(VERSION 3.8)
project(sist_alertatemp_volcan)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
"msg/CustomMsg.msg"
)

add_executable(sensorat1 src/sensorat1.cpp)
ament_target_dependencies(sensorat1 rclcpp std_msgs)

add_executable(lectorsensor src/lectorsensor.cpp)
ament_target_dependencies(lectorsensor rclcpp std_msgs)

rosidl_get_typesupport_target(cpp_typesupport_target
${PROJECT_NAME} rosidl_typesupport_cpp)


target_link_libraries(sensorat1 "${cpp_typesupport_target}")
target_link_libraries(lectorsensor "${cpp_typesupport_target}")

install(TARGETS
  sensorat1
  lectorsensor
DESTINATION lib/${PROJECT_NAME}
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
