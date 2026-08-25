from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        # ── Arguments ──
        DeclareLaunchArgument(
            'serial_port_lidar', default_value='/dev/ttyUSB0',
            description='Serial port for RPLiDAR C1'),
        DeclareLaunchArgument(
            'serial_port_esp32', default_value='/dev/ttyACM0',
            description='Serial port for ESP32-S3'),

        # ── 1. RPLiDAR C1 driver ──
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_node',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': LaunchConfiguration('serial_port_lidar'),
                'serial_baudrate': 460800,
                'frame_id': 'laser',
                'angle_compensate': True,
            }],
            output='screen',
        ),

        # ── 2. Static TF: tilt_link → laser ──
        # (LiDAR is rigidly mounted on the tilting platform)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tilt_to_laser_tf',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.0',
                '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                '--frame-id', 'tilt_link', '--child-frame-id', 'laser'
            ],
        ),

        # ── 3. Tilt angle TF broadcaster (reads ESP32 serial) ──
        # Broadcasts:
        #   - map -> base_link (using base IMU quaternion)
        #   - base_link -> tilt_link (using stepper angle)
        Node(
            package='diy_3d_lidar',
            executable='tilt_tf_broadcaster',
            name='tilt_tf_broadcaster',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port_esp32'),
                'baud_rate': 115200,
                'parent_frame': 'map',
                'child_frame': 'base_link',
            }],
            output='screen',
        ),

        # ── 4. LaserScan → PointCloud2 converter ──
        Node(
            package='diy_3d_lidar',
            executable='scan_to_pointcloud',
            name='scan_to_pointcloud',
            parameters=[{
                'target_frame': 'map',
            }],
            output='screen',
        ),

        # ── 5. Static TF: base_link → thermal_camera_link ──
        # (Defines where the thermal camera is physically mounted relative to the scanner base)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera_tf',
            arguments=[
                '--x', '0.05', '--y', '0.0', '--z', '0.10',
                '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                '--frame-id', 'base_link', '--child-frame-id', 'thermal_camera_link'
            ],
        ),
    ])
