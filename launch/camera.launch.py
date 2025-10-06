import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "device_ip",
            default_value="",
            description="IP address of the camera (for GigE cameras)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "device_serial",
            default_value="",
            description="Serial number of the camera (for USB cameras)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "camera_name",
            default_value="hk_camera",
            description="Name of the camera",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "frame_id",
            default_value="camera_link",
            description="Frame ID of the camera",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "topic_name",
            default_value="image_raw",
            description="Topic name for image publishing",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "exposure_time",
            default_value="20000.0",
            description="Exposure time in microseconds",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gain",
            default_value="1.0",
            description="Camera gain",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "frame_rate",
            default_value="30.0",
            description="Frame rate in FPS",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "pixel_format",
            default_value="BGR8",
            description="Pixel format (Mono8, RGB8, BGR8)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "image_width",
            default_value="1920",
            description="Image width",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "image_height",
            default_value="1080",
            description="Image height",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_transport",
            default_value="false",
            description="Use image transport",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "auto_reconnect",
            default_value="true",
            description="Enable auto reconnection",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "start_rviz",
            default_value="false",
            description="Start RViz2 automatically",
        )
    )

    # Initialize Arguments
    device_ip = LaunchConfiguration("device_ip")
    device_serial = LaunchConfiguration("device_serial")
    camera_name = LaunchConfiguration("camera_name")
    frame_id = LaunchConfiguration("frame_id")
    topic_name = LaunchConfiguration("topic_name")
    exposure_time = LaunchConfiguration("exposure_time")
    gain = LaunchConfiguration("gain")
    frame_rate = LaunchConfiguration("frame_rate")
    pixel_format = LaunchConfiguration("pixel_format")
    image_width = LaunchConfiguration("image_width")
    image_height = LaunchConfiguration("image_height")
    use_transport = LaunchConfiguration("use_transport")
    auto_reconnect = LaunchConfiguration("auto_reconnect")
    start_rviz = LaunchConfiguration("start_rviz")

    # HK Camera node
    hk_camera_node = Node(
        package="hk_camera",
        executable="hk_camera_node_main",
        name=camera_name,
        output="screen",
        parameters=[
            {
                "device_ip": device_ip,
                "device_serial": device_serial,
                "camera_name": camera_name,
                "frame_id": frame_id,
                "topic_name": topic_name,
                "exposure_time": exposure_time,
                "gain": gain,
                "frame_rate": frame_rate,
                "pixel_format": pixel_format,
                "image_width": image_width,
                "image_height": image_height,
                "use_transport": use_transport,
                "auto_reconnect": auto_reconnect,
            }
        ],
    )

    nodes = [
        hk_camera_node,
    ]

    return LaunchDescription(declared_arguments + nodes)