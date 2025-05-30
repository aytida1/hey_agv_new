cmake_minimum_required(VERSION 3.8)
project(hey_agv_new)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/LiftAndLock.action"
  "action/MoveRobot.action"
  "action/DockRobot.action"
  DEPENDENCIES action_msgs geometry_msgs builtin_interfaces
)

add_executable(frame_remapper src/mapper.cpp)
ament_target_dependencies(frame_remapper rclcpp nav_msgs geometry_msgs tf2_ros tf2)

add_executable(scan_merger_v2 src/scan_merger_v2.cpp)
ament_target_dependencies(scan_merger_v2 rclcpp sensor_msgs geometry_msgs tf2_ros tf2)

add_executable(scan_merger src/scan_merger.cpp)
ament_target_dependencies(scan_merger rclcpp sensor_msgs)

add_executable(tag_transform src/tag_transform.cpp)
ament_target_dependencies(tag_transform rclcpp geometry_msgs tf2_ros tf2 tf2_geometry_msgs)



# Passing actions definitions to the gnerator or pipeline
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "action/lift_and_lock.action"
#   DEPENDENCIES action_msgs
# ) 

install (
  PROGRAMS
    src/some.py
    src/dock.py
    src/tag_publisher.py
    DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  frame_remapper
  scan_merger
  scan_merger_v2
  tag_transform
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY src urdf config launch action models
  DESTINATION share/${PROJECT_NAME}
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
