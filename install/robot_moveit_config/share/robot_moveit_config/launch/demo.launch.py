from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = (MoveItConfigsBuilder("Robot", package_name="robot_moveit_config")
                     .robot_description(file_path="config/Robot.urdf.xacro")
                     .robot_description_semantic(file_path="config/Robot.srdf")
                     .to_moveit_configs())
    return generate_demo_launch(moveit_config)
