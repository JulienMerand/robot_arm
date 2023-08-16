from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
 
 
def generate_launch_description():
    ld = LaunchDescription()
 
    camera = Node(
        package="prog_robot",
        executable="camera"
    )

    tictactoe_main = Node(
        package="tictactoe",
        executable="main",
    )

    

    # ld.add_action(rviz_moveit_launch)
    ld.add_action(camera)
    ld.add_action(tictactoe_main)

    return ld