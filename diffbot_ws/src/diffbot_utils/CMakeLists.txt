cmake_minimum_required(VERSION 3.8)
project(diffbot_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

add_executable(trajectory_drawer src/trajectory_drawer.cpp)
ament_target_dependencies(trajectory_drawer rclcpp geometry_msgs nav_msgs)

include_directories(include)

install(
  DIRECTORY include
  DESTINATION include
)

install(TARGETS 
  trajectory_drawer
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(PROGRAMS 
  ${PROJECT_NAME}/imu_converter.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
