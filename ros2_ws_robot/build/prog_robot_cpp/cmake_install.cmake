# Install script for directory: /home/julien/Documents/ros/robot_arm/ros2_ws_robot/src/prog_robot_cpp

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/julien/Documents/ros/robot_arm/ros2_ws_robot/install/prog_robot_cpp")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/prog_robot_cpp/hello_moveit" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/prog_robot_cpp/hello_moveit")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/prog_robot_cpp/hello_moveit"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/prog_robot_cpp" TYPE EXECUTABLE FILES "/home/julien/Documents/ros/robot_arm/ros2_ws_robot/build/prog_robot_cpp/hello_moveit")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/prog_robot_cpp/hello_moveit" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/prog_robot_cpp/hello_moveit")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/prog_robot_cpp/hello_moveit"
         OLD_RPATH "/home/julien/Documents/ros/ws_moveit2/install/moveit_ros_planning_interface/lib:/home/julien/Documents/ros/ws_moveit2/install/moveit_visual_tools/lib:/home/julien/Documents/ros/ws_moveit2/install/moveit_ros_move_group/lib:/home/julien/Documents/ros/microros_ws/install/std_srvs/lib:/home/julien/Documents/ros/ws_moveit2/install/moveit_ros_warehouse/lib:/home/julien/Documents/ros/microros_ws/install/lifecycle_msgs/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/home/julien/Documents/ros/microros_ws/install/nav_msgs/lib:/home/julien/Documents/ros/microros_ws/install/shape_msgs/lib:/home/julien/Documents/ros/microros_ws/install/composition_interfaces/lib:/home/julien/Documents/ros/microros_ws/install/rosgraph_msgs/lib:/home/julien/Documents/ros/microros_ws/install/statistics_msgs/lib:/home/julien/Documents/ros/microros_ws/install/rcl_interfaces/lib:/home/julien/Documents/ros/microros_ws/install/action_msgs/lib:/home/julien/Documents/ros/microros_ws/install/unique_identifier_msgs/lib:/home/julien/Documents/ros/microros_ws/install/trajectory_msgs/lib:/home/julien/Documents/ros/microros_ws/install/visualization_msgs/lib:/home/julien/Documents/ros/microros_ws/install/sensor_msgs/lib:/home/julien/Documents/ros/microros_ws/install/geometry_msgs/lib:/home/julien/Documents/ros/microros_ws/install/std_msgs/lib:/home/julien/Documents/ros/microros_ws/install/builtin_interfaces/lib:/opt/ros/humble/lib:/home/julien/Documents/ros/ws_moveit2/install/moveit_ros_planning/lib:/home/julien/Documents/ros/ws_moveit2/install/moveit_ros_occupancy_map_monitor/lib:/home/julien/Documents/ros/ws_moveit2/install/moveit_core/lib:/home/julien/Documents/ros/ws_moveit2/install/srdfdom/lib:/opt/ros/humble/opt/rviz_ogre_vendor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/prog_robot_cpp/hello_moveit")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/prog_robot_cpp/scene" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/prog_robot_cpp/scene")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/prog_robot_cpp/scene"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/prog_robot_cpp" TYPE EXECUTABLE FILES "/home/julien/Documents/ros/robot_arm/ros2_ws_robot/build/prog_robot_cpp/scene")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/prog_robot_cpp/scene" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/prog_robot_cpp/scene")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/prog_robot_cpp/scene"
         OLD_RPATH "/home/julien/Documents/ros/ws_moveit2/install/moveit_ros_planning_interface/lib:/home/julien/Documents/ros/ws_moveit2/install/moveit_ros_move_group/lib:/home/julien/Documents/ros/microros_ws/install/std_srvs/lib:/home/julien/Documents/ros/ws_moveit2/install/moveit_ros_warehouse/lib:/home/julien/Documents/ros/ws_moveit2/install/moveit_ros_planning/lib:/home/julien/Documents/ros/ws_moveit2/install/moveit_ros_occupancy_map_monitor/lib:/home/julien/Documents/ros/ws_moveit2/install/moveit_core/lib:/home/julien/Documents/ros/microros_ws/install/lifecycle_msgs/lib:/home/julien/Documents/ros/microros_ws/install/visualization_msgs/lib:/home/julien/Documents/ros/microros_ws/install/sensor_msgs/lib:/home/julien/Documents/ros/microros_ws/install/shape_msgs/lib:/home/julien/Documents/ros/microros_ws/install/trajectory_msgs/lib:/home/julien/Documents/ros/microros_ws/install/rosgraph_msgs/lib:/home/julien/Documents/ros/microros_ws/install/statistics_msgs/lib:/home/julien/Documents/ros/microros_ws/install/rcl_interfaces/lib:/home/julien/Documents/ros/microros_ws/install/action_msgs/lib:/home/julien/Documents/ros/microros_ws/install/unique_identifier_msgs/lib:/home/julien/Documents/ros/microros_ws/install/geometry_msgs/lib:/home/julien/Documents/ros/microros_ws/install/std_msgs/lib:/home/julien/Documents/ros/microros_ws/install/builtin_interfaces/lib:/opt/ros/humble/lib:/home/julien/Documents/ros/ws_moveit2/install/srdfdom/lib:/opt/ros/humble/lib/x86_64-linux-gnu:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/prog_robot_cpp/scene")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/prog_robot_cpp/move" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/prog_robot_cpp/move")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/prog_robot_cpp/move"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/prog_robot_cpp" TYPE EXECUTABLE FILES "/home/julien/Documents/ros/robot_arm/ros2_ws_robot/build/prog_robot_cpp/move")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/prog_robot_cpp/move" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/prog_robot_cpp/move")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/prog_robot_cpp/move"
         OLD_RPATH "/home/julien/Documents/ros/ws_moveit2/install/moveit_ros_planning_interface/lib:/home/julien/Documents/ros/ws_moveit2/install/moveit_visual_tools/lib:/home/julien/Documents/ros/ws_moveit2/install/moveit_ros_move_group/lib:/home/julien/Documents/ros/microros_ws/install/std_srvs/lib:/home/julien/Documents/ros/ws_moveit2/install/moveit_ros_warehouse/lib:/home/julien/Documents/ros/microros_ws/install/lifecycle_msgs/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/home/julien/Documents/ros/microros_ws/install/nav_msgs/lib:/home/julien/Documents/ros/microros_ws/install/shape_msgs/lib:/home/julien/Documents/ros/microros_ws/install/composition_interfaces/lib:/home/julien/Documents/ros/microros_ws/install/rosgraph_msgs/lib:/home/julien/Documents/ros/microros_ws/install/statistics_msgs/lib:/home/julien/Documents/ros/microros_ws/install/rcl_interfaces/lib:/home/julien/Documents/ros/microros_ws/install/action_msgs/lib:/home/julien/Documents/ros/microros_ws/install/unique_identifier_msgs/lib:/home/julien/Documents/ros/microros_ws/install/trajectory_msgs/lib:/home/julien/Documents/ros/microros_ws/install/visualization_msgs/lib:/home/julien/Documents/ros/microros_ws/install/sensor_msgs/lib:/home/julien/Documents/ros/microros_ws/install/geometry_msgs/lib:/home/julien/Documents/ros/microros_ws/install/std_msgs/lib:/home/julien/Documents/ros/microros_ws/install/builtin_interfaces/lib:/opt/ros/humble/lib:/home/julien/Documents/ros/ws_moveit2/install/moveit_ros_planning/lib:/home/julien/Documents/ros/ws_moveit2/install/moveit_ros_occupancy_map_monitor/lib:/home/julien/Documents/ros/ws_moveit2/install/moveit_core/lib:/home/julien/Documents/ros/ws_moveit2/install/srdfdom/lib:/opt/ros/humble/opt/rviz_ogre_vendor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/prog_robot_cpp/move")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/prog_robot_cpp/frame_effector" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/prog_robot_cpp/frame_effector")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/prog_robot_cpp/frame_effector"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/prog_robot_cpp" TYPE EXECUTABLE FILES "/home/julien/Documents/ros/robot_arm/ros2_ws_robot/build/prog_robot_cpp/frame_effector")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/prog_robot_cpp/frame_effector" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/prog_robot_cpp/frame_effector")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/prog_robot_cpp/frame_effector"
         OLD_RPATH "/home/julien/Documents/ros/microros_ws/install/rosgraph_msgs/lib:/home/julien/Documents/ros/microros_ws/install/statistics_msgs/lib:/home/julien/Documents/ros/microros_ws/install/rcl_interfaces/lib:/home/julien/Documents/ros/microros_ws/install/geometry_msgs/lib:/home/julien/Documents/ros/microros_ws/install/std_msgs/lib:/home/julien/Documents/ros/microros_ws/install/action_msgs/lib:/home/julien/Documents/ros/microros_ws/install/builtin_interfaces/lib:/home/julien/Documents/ros/microros_ws/install/unique_identifier_msgs/lib:/opt/ros/humble/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/prog_robot_cpp/frame_effector")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/prog_robot_cpp/transformation_matrix_calculator" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/prog_robot_cpp/transformation_matrix_calculator")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/prog_robot_cpp/transformation_matrix_calculator"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/prog_robot_cpp" TYPE EXECUTABLE FILES "/home/julien/Documents/ros/robot_arm/ros2_ws_robot/build/prog_robot_cpp/transformation_matrix_calculator")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/prog_robot_cpp/transformation_matrix_calculator" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/prog_robot_cpp/transformation_matrix_calculator")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/prog_robot_cpp/transformation_matrix_calculator"
         OLD_RPATH "/home/julien/Documents/ros/ws_moveit2/install/moveit_ros_planning_interface/lib:/home/julien/Documents/ros/ws_moveit2/install/moveit_ros_move_group/lib:/home/julien/Documents/ros/microros_ws/install/std_srvs/lib:/home/julien/Documents/ros/ws_moveit2/install/moveit_ros_warehouse/lib:/home/julien/Documents/ros/ws_moveit2/install/moveit_ros_planning/lib:/home/julien/Documents/ros/ws_moveit2/install/moveit_ros_occupancy_map_monitor/lib:/home/julien/Documents/ros/ws_moveit2/install/moveit_core/lib:/home/julien/Documents/ros/microros_ws/install/lifecycle_msgs/lib:/home/julien/Documents/ros/microros_ws/install/visualization_msgs/lib:/home/julien/Documents/ros/microros_ws/install/sensor_msgs/lib:/home/julien/Documents/ros/microros_ws/install/shape_msgs/lib:/home/julien/Documents/ros/microros_ws/install/trajectory_msgs/lib:/home/julien/Documents/ros/microros_ws/install/rosgraph_msgs/lib:/home/julien/Documents/ros/microros_ws/install/statistics_msgs/lib:/home/julien/Documents/ros/microros_ws/install/rcl_interfaces/lib:/home/julien/Documents/ros/microros_ws/install/action_msgs/lib:/home/julien/Documents/ros/microros_ws/install/unique_identifier_msgs/lib:/home/julien/Documents/ros/microros_ws/install/geometry_msgs/lib:/home/julien/Documents/ros/microros_ws/install/std_msgs/lib:/home/julien/Documents/ros/microros_ws/install/builtin_interfaces/lib:/opt/ros/humble/lib:/home/julien/Documents/ros/ws_moveit2/install/srdfdom/lib:/opt/ros/humble/lib/x86_64-linux-gnu:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/prog_robot_cpp/transformation_matrix_calculator")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/prog_robot_cpp" TYPE DIRECTORY FILES "/home/julien/Documents/ros/robot_arm/ros2_ws_robot/src/prog_robot_cpp/launch")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/julien/Documents/ros/robot_arm/ros2_ws_robot/build/prog_robot_cpp/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/prog_robot_cpp")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/julien/Documents/ros/robot_arm/ros2_ws_robot/build/prog_robot_cpp/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/prog_robot_cpp")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/prog_robot_cpp/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/prog_robot_cpp/environment" TYPE FILE FILES "/home/julien/Documents/ros/robot_arm/ros2_ws_robot/build/prog_robot_cpp/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/prog_robot_cpp/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/prog_robot_cpp/environment" TYPE FILE FILES "/home/julien/Documents/ros/robot_arm/ros2_ws_robot/build/prog_robot_cpp/ament_cmake_environment_hooks/path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/prog_robot_cpp" TYPE FILE FILES "/home/julien/Documents/ros/robot_arm/ros2_ws_robot/build/prog_robot_cpp/ament_cmake_environment_hooks/local_setup.bash")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/prog_robot_cpp" TYPE FILE FILES "/home/julien/Documents/ros/robot_arm/ros2_ws_robot/build/prog_robot_cpp/ament_cmake_environment_hooks/local_setup.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/prog_robot_cpp" TYPE FILE FILES "/home/julien/Documents/ros/robot_arm/ros2_ws_robot/build/prog_robot_cpp/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/prog_robot_cpp" TYPE FILE FILES "/home/julien/Documents/ros/robot_arm/ros2_ws_robot/build/prog_robot_cpp/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/prog_robot_cpp" TYPE FILE FILES "/home/julien/Documents/ros/robot_arm/ros2_ws_robot/build/prog_robot_cpp/ament_cmake_environment_hooks/package.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/julien/Documents/ros/robot_arm/ros2_ws_robot/build/prog_robot_cpp/ament_cmake_index/share/ament_index/resource_index/packages/prog_robot_cpp")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/prog_robot_cpp/cmake" TYPE FILE FILES
    "/home/julien/Documents/ros/robot_arm/ros2_ws_robot/build/prog_robot_cpp/ament_cmake_core/prog_robot_cppConfig.cmake"
    "/home/julien/Documents/ros/robot_arm/ros2_ws_robot/build/prog_robot_cpp/ament_cmake_core/prog_robot_cppConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/prog_robot_cpp" TYPE FILE FILES "/home/julien/Documents/ros/robot_arm/ros2_ws_robot/src/prog_robot_cpp/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/julien/Documents/ros/robot_arm/ros2_ws_robot/build/prog_robot_cpp/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
