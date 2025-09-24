cmake_minimum_required(VERSION 3.8)
project(rover_assignment)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(BZip2 REQUIRED)
find_package(rcl_interfaces REQUIRED)
include_directories(include)



add_executable(simple_subscriber src/bz2_subscriber.cpp)
ament_target_dependencies(simple_subscriber rclcpp std_msgs rcl_interfaces)
target_link_libraries(simple_subscriber BZip2::BZip2)

ament_python_install_package(${PROJECT_NAME})
install(
  DIRECTORY include
  DESTINATION include
)

install(TARGETS 
  simple_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  ${PROJECT_NAME}/bz2_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
