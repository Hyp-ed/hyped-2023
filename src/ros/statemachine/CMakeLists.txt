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
find_package(Boost REQUIRED)
find_package(HypedCore REQUIRED)
find_package(HypedIO REQUIRED)
find_package(HypedMotorControllers REQUIRED)
find_package(HypedNavigation REQUIRED)
find_package(HypedSensors REQUIRED)
find_package(HypedStateMachine REQUIRED)
find_package(hyped_messages REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

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

add_executable(stm1 src/stm1.cpp)
ament_target_dependencies(stm1 rclcpp std_msgs)
target_link_libraries(
  stm1
  Hyped::hyped_core
  Hyped::hyped_statemachine
)

add_executable(stm2 src/stm2.cpp)
ament_target_dependencies(stm2 rclcpp std_msgs)
target_link_libraries(
  stm2
  Hyped::hyped_core
  Hyped::hyped_statemachine
)

add_executable(stm3 src/stm3.cpp)
ament_target_dependencies(stm3 rclcpp std_msgs)
target_link_libraries(
  stm3
  Hyped::hyped_core
  Hyped::hyped_statemachine
)

add_executable(stm4 src/stm4.cpp)
ament_target_dependencies(stm4 rclcpp std_msgs)
target_link_libraries(
  stm4
  Hyped::hyped_core
  Hyped::hyped_statemachine
)

add_executable(transitionpublisher src/transitionpublisher.cpp)
ament_target_dependencies(transitionpublisher rclcpp std_msgs)
target_link_libraries(
  transitionpublisher
  Hyped::hyped_core
)

install(TARGETS
  stm1
  stm2
  stm3
  stm4
  DESTINATION lib/${PROJECT_NAME})

ament_package()

