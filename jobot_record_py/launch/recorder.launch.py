from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 定义可配置参数
    device_index_arg = DeclareLaunchArgument(
        "device_index", default_value="0",
        description="Audio device index"
    )
    sample_rate_arg = DeclareLaunchArgument(
        "sample_rate", default_value="44100",
        description="Sample rate of audio"
    )
    channels_arg = DeclareLaunchArgument(
        "channels", default_value="1",
        description="Number of audio channels"
    )
    frame_size_arg = DeclareLaunchArgument(
        "frame_size", default_value="1024",
        description="Frame size"
    )
    format_arg = DeclareLaunchArgument(
        "format", default_value="pcm16",
        description="Audio format"
    )

    # 启动节点
    recorder_node = Node(
        package='jobot_record_py',
        executable='recorder_node',
        name='recorder_node',
        output='screen',
        parameters=[{
            "device_index": LaunchConfiguration("device_index"),
            "sample_rate": LaunchConfiguration("sample_rate"),
            "channels": LaunchConfiguration("channels"),
            "frame_size": LaunchConfiguration("frame_size"),
            "format": LaunchConfiguration("format"),
        }]
    )

    return LaunchDescription([
        device_index_arg,
        sample_rate_arg,
        channels_arg,
        frame_size_arg,
        format_arg,
        recorder_node
    ])
