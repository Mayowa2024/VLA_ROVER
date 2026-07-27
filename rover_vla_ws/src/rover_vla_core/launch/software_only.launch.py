from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    instruction = LaunchConfiguration("instruction")

    return LaunchDescription([
        DeclareLaunchArgument(
            "instruction",
            default_value="Drive to the blue cone",
            description="Task instruction for the episode",
        ),
        Node(
            package="rover_vla_core",
            executable="instruction_publisher",
            name="instruction_publisher",
            output="screen",
            parameters=[{"instruction": instruction}],
        ),
        Node(
            package="rover_vla_core",
            executable="safety_filter",
            name="safety_filter",
            output="screen",
            parameters=[{
                "input_topic": "/vla/raw_cmd_vel",
                "output_topic": "/cmd_vel",
                "max_linear_mps": 0.10,
                "max_angular_rps": 0.40,
                "allow_reverse": False,
                "timeout_sec": 0.50,
            }],
        ),
        Node(
            package="rover_vla_core",
            executable="fake_odom",
            name="fake_odom",
            output="screen",
            parameters=[{
                "cmd_topic": "/cmd_vel",
                "odom_topic": "/odom",
                "publish_tf": False,
            }],
        ),
        Node(
            package="rover_vla_core",
            executable="mock_motor_driver",
            name="mock_motor_driver",
            output="screen",
            parameters=[{
                "cmd_topic": "/cmd_vel",
                "wheel_command_topic": "/rover/mock_wheel_command",
            }],
        ),
    ])
