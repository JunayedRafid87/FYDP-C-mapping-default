from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # ── Arguments ──
        DeclareLaunchArgument(
            'enable_motion_gating', default_value='False',
            description='Enable/disable mapping pause on motion detection'),
        DeclareLaunchArgument(
            'lidar_only', default_value='True',
            description='Run human detection in Lidar-Only mode'),
        DeclareLaunchArgument(
            'invert_z', default_value='False',
            description='Invert Z axis to fix upside-down height map'),

        # ── 1. LaserScan → PointCloud2 Converter & Map Accumulator ──
        Node(
            package='diy_3d_lidar',
            executable='scan_to_pointcloud',
            name='scan_to_pointcloud',
            parameters=[{
                'target_frame': 'map',
                'enable_motion_gating': LaunchConfiguration('enable_motion_gating'),
                'invert_z': LaunchConfiguration('invert_z'),
            }],
            output='screen',
        ),

        # ── 2. Multimodal Fusion / Lidar-Only Human Detector ──
        Node(
            package='diy_3d_lidar',
            executable='multimodal_fusion_node',
            name='multimodal_fusion_node',
            parameters=[{
                'lidar_only': LaunchConfiguration('lidar_only'),
            }],
            output='screen',
        ),
    ])
