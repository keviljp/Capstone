from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory('swarm_state_estimation'))
    cfg = share / 'config'
    return LaunchDescription([
        DeclareLaunchArgument('video_device', default_value='/dev/video0'),
        Node(
            package='swarm_state_estimation',
            executable='usb_camera_node',
            name='usb_camera_node',
            output='screen',
            parameters=[{
                'video_device': LaunchConfiguration('video_device'),
                'width': 1280,
                'height': 720,
                'fps': 30.0,
                'frame_id': 'map',
                'camera_name': 'overhead_usb_cam',
                'publish_rectified': True,
                'calibration_file': str(cfg / 'camera_info' / 'overhead_camera.yaml'),
            }],
        ),
    ])
