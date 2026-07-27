from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bind_address = LaunchConfiguration("bind_address")
    udp_port = LaunchConfiguration("udp_port")
    allowed_source_ip = LaunchConfiguration("allowed_source_ip")
    serial_port = LaunchConfiguration("serial_port")
    baud_rate = LaunchConfiguration("baud_rate")
    enable_motors = LaunchConfiguration("enable_motors")
    use_fake_odom = LaunchConfiguration("use_fake_odom")
    encoder_serial_port = LaunchConfiguration("encoder_serial_port")
    encoder_baud_rate = LaunchConfiguration("encoder_baud_rate")
    encoder_counts_per_revolution = LaunchConfiguration(
        "encoder_counts_per_revolution"
    )
    encoder_wheel_diameter_m = LaunchConfiguration(
        "encoder_wheel_diameter_m"
    )
    encoder_track_width_m = LaunchConfiguration("encoder_track_width_m")
    encoder_left_direction = LaunchConfiguration("encoder_left_direction")
    encoder_right_direction = LaunchConfiguration("encoder_right_direction")
    encoder_measurement_timeout_sec = LaunchConfiguration(
        "encoder_measurement_timeout_sec"
    )
    encoder_max_tick_jump = LaunchConfiguration("encoder_max_tick_jump")
    encoder_max_wheel_speed_mps = LaunchConfiguration(
        "encoder_max_wheel_speed_mps"
    )
    odom_frame = LaunchConfiguration("odom_frame")
    base_frame = LaunchConfiguration("base_frame")
    publish_tf = LaunchConfiguration("publish_tf")

    return LaunchDescription([
        DeclareLaunchArgument(
            "bind_address",
            default_value="0.0.0.0",
            description="Local IPv4 address used by the UDP teleop receiver",
        ),
        DeclareLaunchArgument(
            "udp_port",
            default_value="6001",
            description="UDP teleop port",
        ),
        DeclareLaunchArgument(
            "allowed_source_ip",
            default_value="",
            description="Optional single IPv4 address allowed to send commands",
        ),
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/ttyACM0",
            description="Motor-controller serial device",
        ),
        DeclareLaunchArgument(
            "baud_rate",
            default_value="9600",
            description="Motor-controller serial baud rate",
        ),
        DeclareLaunchArgument(
            "enable_motors",
            default_value="false",
            choices=["true", "false"],
            description="Explicitly opt in to opening the real motor serial port",
        ),
        DeclareLaunchArgument(
            "use_fake_odom",
            default_value="false",
            choices=["true", "false"],
            description="Use command-integrated odometry instead of encoders",
        ),
        DeclareLaunchArgument(
            "encoder_serial_port",
            default_value="/dev/ttyUSB0",
            description="Encoder Arduino serial device",
        ),
        DeclareLaunchArgument(
            "encoder_baud_rate",
            default_value="115200",
            description="Encoder Arduino serial baud rate",
        ),
        DeclareLaunchArgument(
            "encoder_counts_per_revolution",
            default_value="1224.0",
            description="Quadrature counts per wheel revolution",
        ),
        DeclareLaunchArgument(
            "encoder_wheel_diameter_m",
            default_value="0.15875",
            description="Wheel diameter in metres",
        ),
        DeclareLaunchArgument(
            "encoder_track_width_m",
            default_value="0.352425",
            description="Effective left-to-right track width in metres",
        ),
        DeclareLaunchArgument(
            "encoder_left_direction",
            default_value="1.0",
            choices=["1.0", "-1.0"],
            description="Rear-left encoder direction multiplier",
        ),
        DeclareLaunchArgument(
            "encoder_right_direction",
            default_value="1.0",
            choices=["1.0", "-1.0"],
            description="Front-right encoder direction multiplier",
        ),
        DeclareLaunchArgument(
            "encoder_measurement_timeout_sec",
            default_value="0.5",
            description="Time without counts before publishing zero velocity",
        ),
        DeclareLaunchArgument(
            "encoder_max_tick_jump",
            default_value="4096",
            description="Absolute count-jump corruption/reset guard",
        ),
        DeclareLaunchArgument(
            "encoder_max_wheel_speed_mps",
            default_value="2.0",
            description="Largest physically accepted encoder wheel speed",
        ),
        DeclareLaunchArgument(
            "odom_frame",
            default_value="odom",
            description="Odometry reference frame",
        ),
        DeclareLaunchArgument(
            "base_frame",
            default_value="base_link",
            description="Rover base frame",
        ),
        DeclareLaunchArgument(
            "publish_tf",
            default_value="false",
            choices=["true", "false"],
            description="Publish odom to base_link TF",
        ),
        LogInfo(
            condition=UnlessCondition(enable_motors),
            msg=(
                "Motor output is disabled. Inspect /teleop/cmd_vel and /cmd_vel; "
                "set enable_motors:=true only after bench validation."
            ),
        ),
        Node(
            package="rover_vla_core",
            executable="teleop_adapter",
            name="teleop_adapter",
            output="screen",
            parameters=[{
                "bind_address": bind_address,
                "port": ParameterValue(udp_port, value_type=int),
                "allowed_source_ip": allowed_source_ip,
                "output_topic": "/teleop/cmd_vel",
            }],
        ),
        Node(
            package="rover_vla_core",
            executable="safety_filter",
            name="safety_filter",
            output="screen",
            parameters=[{
                "input_topic": "/teleop/cmd_vel",
                "output_topic": "/cmd_vel",
                "max_linear_mps": 0.10,
                "max_angular_rps": 0.40,
                "allow_reverse": False,
                "timeout_sec": 0.50,
            }],
        ),
        Node(
            condition=IfCondition(use_fake_odom),
            package="rover_vla_core",
            executable="fake_odom",
            name="fake_odom",
            output="screen",
            parameters=[{
                "cmd_topic": "/cmd_vel",
                "odom_topic": "/odom",
                "state_topic": "/vla/state",
                "odom_frame": odom_frame,
                "base_frame": base_frame,
                "publish_tf": ParameterValue(
                    publish_tf, value_type=bool
                ),
            }],
        ),
        Node(
            condition=UnlessCondition(use_fake_odom),
            package="rover_vla_core",
            executable="encoder_odom",
            name="encoder_odom",
            output="screen",
            parameters=[{
                "serial_port": encoder_serial_port,
                "baud_rate": ParameterValue(
                    encoder_baud_rate, value_type=int
                ),
                "counts_per_revolution": ParameterValue(
                    encoder_counts_per_revolution, value_type=float
                ),
                "wheel_diameter_m": ParameterValue(
                    encoder_wheel_diameter_m, value_type=float
                ),
                "track_width_m": ParameterValue(
                    encoder_track_width_m, value_type=float
                ),
                "left_direction": ParameterValue(
                    encoder_left_direction, value_type=float
                ),
                "right_direction": ParameterValue(
                    encoder_right_direction, value_type=float
                ),
                "measurement_timeout_sec": ParameterValue(
                    encoder_measurement_timeout_sec, value_type=float
                ),
                "max_tick_jump": ParameterValue(
                    encoder_max_tick_jump, value_type=int
                ),
                "max_wheel_speed_mps": ParameterValue(
                    encoder_max_wheel_speed_mps, value_type=float
                ),
                "odom_topic": "/odom",
                "state_topic": "/vla/state",
                "odom_frame": odom_frame,
                "base_frame": base_frame,
                "publish_tf": ParameterValue(
                    publish_tf, value_type=bool
                ),
            }],
        ),
        Node(
            condition=IfCondition(enable_motors),
            package="rover_vla_core",
            executable="serial_motor_driver",
            name="serial_motor_driver",
            output="screen",
            parameters=[{
                "cmd_topic": "/cmd_vel",
                "serial_port": serial_port,
                "baud_rate": ParameterValue(baud_rate, value_type=int),
            }],
        ),
    ])
