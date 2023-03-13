cmake_minimum_required(VERSION 3.8)
project(statemachine)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} /home/hyped/build/export)
find_package(Eigen3 REQUIRED)
find_package(HypedCore REQUIRED)
find_package(HypedIO REQUIRED)
find_package(HypedMotorControllers REQUIRED)
find_package(HypedNavigation REQUIRED)
find_package(HypedSensors REQUIRED)
find_package(hyped_messages REQUIRED)

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

add_executable(publisherTest src/publisherTest.cpp)
ament_target_dependencies(publisherTest rclcpp std_msgs)
target_link_libraries(
  publisherTest
  Hyped::hyped_core
)

add_executable(subscriberTest src/subscriberTest.cpp)
ament_target_dependencies(subscriberTest rclcpp std_msgs)
target_link_libraries(
  subscriberTest
  Hyped::hyped_core
)


install(TARGETS
  publisherTest
  subscriberTest
  DESTINATION lib/${PROJECT_NAME})

ament_package()

