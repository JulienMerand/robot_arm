from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
 
 
def generate_launch_description():
    ld = LaunchDescription()

    # rviz_moveit_launch = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource([
    #                 FindPackageShare("robot_moveit_config"), '/launch', '/demo.launch.py'])
    #         )
 
    frame = Node(
        package="prog_robot_cpp",
        executable="frame_effector"
    )

    move_cpp = Node(
        package="prog_robot_cpp",
        executable="move",
    )

    

    # ld.add_action(rviz_moveit_launch)
    ld.add_action(frame)
    ld.add_action(move_cpp)

    return ld