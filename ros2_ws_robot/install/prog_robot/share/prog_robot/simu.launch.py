from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
 
 
def generate_launch_description():
    ld = LaunchDescription()

    rviz_moveit_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("robot_moveit_config"), '/launch', '/demo.launch.py'])
            )
 
    pub_to_arduino_node = Node(
        package="prog_robot",
        executable="pub_to_arduino",
        # name='pub_to_arduino'
    )

    scene_node = Node(
        package="prog_robot_cpp",
        executable="scene"
    )

    ld.add_action(rviz_moveit_launch)
    ld.add_action(pub_to_arduino_node)
    ld.add_action(scene_node)

    return ld

