cmake_minimum_required(VERSION 3.8)
project(custom_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate ROS 2 interfaces (messages, services, actions)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TemperatureMsg.msg"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)  # Moved before ament_package()
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
