cmake_minimum_required(VERSION 3.8)
project(prog_robot_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_visual_tools REQUIRED)

add_executable(hello_moveit src/hello_moveit.cpp)
ament_target_dependencies(
  hello_moveit
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "rclcpp"
)

add_executable(move src/move.cpp)
ament_target_dependencies(
  move
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "rclcpp"
  "tf2_ros"
  "geometry_msgs"
)

add_executable(transformation_matrix_calculator src/transformation_matrix_calculator.cpp)
ament_target_dependencies(
  transformation_matrix_calculator
  "rclcpp"
  "tf2"
  "tf2_ros"
  "moveit_ros_planning_interface"
)

add_executable(scene src/scene.cpp)
ament_target_dependencies(
  scene
  "moveit_ros_planning_interface"
  "rclcpp"
)

add_executable(frame_effector src/frame_effector.cpp)
ament_target_dependencies(
  frame_effector
  "geometry_msgs"
  "rclcpp"
  "tf2_ros"
)

install(TARGETS 
  hello_moveit 
  scene
  move
  frame_effector
  transformation_matrix_calculator
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
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
