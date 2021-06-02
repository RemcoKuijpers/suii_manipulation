cmake_minimum_required(VERSION 3.5)
project(suii_manipulation)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(ur_rtde REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(main src/main.cpp src/robotControl.cpp)
add_executable(joint_state_publisher src/joint_state_publisher.cpp)

target_link_libraries(main ur_rtde::rtde)
target_link_libraries(joint_state_publisher ur_rtde::rtde)


ament_target_dependencies(main rclcpp Boost)
ament_target_dependencies(joint_state_publisher rclcpp Boost sensor_msgs)

install(TARGETS
  main joint_state_publisher
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  launch
  urdf
  meshes
  rviz
  config
  resources
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
