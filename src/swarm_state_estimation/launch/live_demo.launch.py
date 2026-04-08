from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory('swarm_state_estimation'))
    cfg = share / 'config'
    rviz_cfg = share / 'rviz' / 'swarm_state.rviz'

    use_rviz = LaunchConfiguration('use_rviz')
    video_device = LaunchConfiguration('video_device')

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('video_device', default_value='/dev/video3'),

        Node(
            package='swarm_state_estimation',
            executable='usb_camera_node',
            name='usb_camera_node',
            output='screen',
            parameters=[{
                'video_device': video_device,
                'width': 1280,
                'height': 720,
                'fps': 30.0,
                'frame_id': 'map',
                'camera_name': 'overhead_usb_cam',
                'publish_rectified': True,
                'calibration_file': str(cfg / 'camera_info' / 'overhead_camera.yaml'),
            }],
        ),

        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag',
            output='screen',
            parameters=[str(cfg / 'apriltag_36h11.yaml')],
            remappings=[
                ('image_rect', '/camera/image_rect'),
                ('camera_info', '/camera/camera_info'),
                ('detections', '/detections'),
            ],
        ),

        Node(
            package='swarm_state_estimation',
            executable='tag_router',
            name='tag_router',
            output='screen',
            parameters=[{
                'tag_map_file': str(cfg / 'tag_map.yaml'),
                'detections_topic': '/detections',
                'map_frame': 'map',
                'x_variance': 0.01,
                'y_variance': 0.01,
                'yaw_variance': 0.03,
                'ignore_unknown_tags': True,
            }],
        ),

        Node(
            package='swarm_state_estimation',
            executable='ukf_manager',
            name='ukf_manager',
            output='screen',
            parameters=[{
                'ukf_template_file': str(cfg / 'ukf_base.yaml'),
                'discovery_topic': '/swarm_state/discovered_robot_id',
            }],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', str(rviz_cfg)],
            condition=IfCondition(use_rviz),
        ),
    ])
