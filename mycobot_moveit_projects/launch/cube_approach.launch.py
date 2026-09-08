"""Launch vision-guided cube pick-and-place.

Requires:
    1. gazebo_sim.launch.py
    2. gazebo_move_group.launch.py

The selected detector publishes:
    /selected_cube/pixel_center

pixel_to_world converts this into:
    /selected_cube/world_center

cube_approach then performs the pick-and-place sequence.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import LaunchConfigurationEquals
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder

try:
    from mycobot_yolo_assets.paths import default_model_path
except ImportError:
    default_model_path = lambda: (  # noqa: E731
        "/root/mycobot_ws/install/mycobot_yolo_assets/share/"
        "mycobot_yolo_assets/assets/runs/cube_detector/weights/best.pt"
    )


def _on_cube_exit(event, context):
    """Stop the remaining perception nodes after cube_approach exits."""
    del context

    return [
        EmitEvent(
            event=Shutdown(
                reason=(
                    f"cube_approach finished with exit code "
                    f"{event.returncode}"
                )
            )
        )
    ]


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            "mycobot_280jn_sim",
            package_name="mycobot_280jn_moveit_config",
        )
        .to_moveit_configs()
    )

    detector = LaunchConfiguration("detector")
    target_color = LaunchConfiguration("target_color")
    model_path = LaunchConfiguration("model_path")
    confidence_threshold = LaunchConfiguration(
        "confidence_threshold"
    )
    image_size = LaunchConfiguration("image_size")

    # Classical HSV detector retained as a fallback.
    hsv_detector_node = Node(
        package="mycobot_sim_projects",
        executable="color_cube_detector",
        name="color_cube_detector",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "target_color": target_color,
            }
        ],
        condition=LaunchConfigurationEquals(
            "detector",
            "hsv",
        ),
    )

    # Trained YOLO11 detector.
    yolo_detector_node = Node(
        package="mycobot_sim_projects",
        executable="yolo_cube_detector",
        name="yolo_cube_detector",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "target_color": target_color,
                "model_path": model_path,
                "confidence_threshold": ParameterValue(
                    confidence_threshold,
                    value_type=float,
                ),
                "image_size": ParameterValue(
                    image_size,
                    value_type=int,
                ),
            }
        ],
        condition=LaunchConfigurationEquals(
            "detector",
            "yolo",
        ),
    )

    pixel_to_world_node = Node(
        package="mycobot_sim_projects",
        executable="pixel_to_world",
        name="pixel_to_world",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
            }
        ],
    )

    cube_approach_node = Node(
        package="mycobot_moveit_projects",
        executable="cube_approach",
        name="cube_approach",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": True,
                "place_x": ParameterValue(
                    LaunchConfiguration("place_x"),
                    value_type=float,
                ),
                "place_y": ParameterValue(
                    LaunchConfiguration("place_y"),
                    value_type=float,
                ),
                "place_z": ParameterValue(
                    LaunchConfiguration("place_z"),
                    value_type=float,
                ),
                "place_descend_z": ParameterValue(
                    LaunchConfiguration("place_descend_z"),
                    value_type=float,
                ),
                "return_home": ParameterValue(
                    LaunchConfiguration("return_home"),
                    value_type=bool,
                ),
            },
        ],
    )

    stop_after_pick = RegisterEventHandler(
        OnProcessExit(
            target_action=cube_approach_node,
            on_exit=_on_cube_exit,
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "detector",
                default_value="yolo",
                description="Cube detector: yolo or hsv.",
            ),
            DeclareLaunchArgument(
                "model_path",
                default_value=default_model_path(),
                description="YOLO model weights path.",
            ),
            DeclareLaunchArgument(
                "confidence_threshold",
                default_value="0.50",
                description="Minimum YOLO detection confidence.",
            ),
            DeclareLaunchArgument(
                "image_size",
                default_value="416",
                description="YOLO inference image size.",
            ),
            DeclareLaunchArgument(
                "target_color",
                default_value="red",
                description=(
                    "Cube colour to select: red, green, or blue."
                ),
            ),
            DeclareLaunchArgument(
                "place_x",
                default_value="0.10",
                description="Placement approach X in world frame.",
            ),
            DeclareLaunchArgument(
                "place_y",
                default_value="0.15",
                description="Placement approach Y in world frame.",
            ),
            DeclareLaunchArgument(
                "place_z",
                default_value="0.140",
                description="Placement carry/retract TCP height.",
            ),
            DeclareLaunchArgument(
                "place_descend_z",
                default_value="0.055",
                description="Placement release TCP height.",
            ),
            DeclareLaunchArgument(
                "return_home",
                default_value="true",
                description="Return to the SRDF home position.",
            ),
            hsv_detector_node,
            yolo_detector_node,
            pixel_to_world_node,
            TimerAction(
                period=2.0,
                actions=[cube_approach_node],
            ),
            stop_after_pick,
        ]
    )