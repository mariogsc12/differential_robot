cmake_minimum_required(VERSION 3.8)
project(diffbot_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# C++
add_executable(simple_lifecycle_node src/simple_lifecycle_node.cpp)
ament_target_dependencies(simple_lifecycle_node rclcpp rclcpp_lifecycle std_msgs)

install(TARGETS
  simple_lifecycle_node
  DESTINATION lib/${PROJECT_NAME}
)

# Python
install(PROGRAMS 
  ${PROJECT_NAME}/simple_lifecycle_node.py
  ${PROJECT_NAME}/simple_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
