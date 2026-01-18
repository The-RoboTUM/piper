from launch import LaunchDescription
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch

from moveit


def generate_launch_description():
    # Build MoveIt config
    moveit_config = (
        MoveItConfigsBuilder("piper", package_name="piper_with_gripper_moveit")
        .to_moveit_configs()
    )

    # Standard MoveIt demo launch (RViz, move_group, etc.)
    demo_launch = generate_demo_launch(moveit_config)

    # Your C++ node
    pose_goal_node = Node(
        package="piper_with_gripper_moveit",
        executable="piper_pose_goal",
        name="piper_pose_goal",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # Combine MoveIt demo + your node
    return LaunchDescription(demo_launch.entities + [pose_goal_node])
