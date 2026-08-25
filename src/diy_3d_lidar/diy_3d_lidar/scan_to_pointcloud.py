#!/usr/bin/env python3
"""
Scan to PointCloud2 Converter & Map Accumulator
==============================================
Converts 2D LaserScan messages into 3D PointCloud2 using TF transforms,
voxel-filters them, and accumulates them into a persistent 3D map.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener
from laser_geometry import LaserProjection
import tf2_sensor_msgs
import sensor_msgs_py.point_cloud2 as pc2
from std_srvs.srv import Trigger
import os


class ScanToPointCloud(Node):
    def __init__(self):
        super().__init__('scan_to_pointcloud')

        self.declare_parameter('target_frame', 'map')
        self.target_frame = self.get_parameter('target_frame').value

        # Parameters for point cloud accumulation and saving
        self.declare_parameter('voxel_size', 0.02)
        self.declare_parameter('save_filename', 'map.pcd')
        self.declare_parameter('enable_motion_gating', False)
        self.declare_parameter('invert_z', False)

        self.voxel_size = self.get_parameter('voxel_size').value
        self.save_filename = self.get_parameter('save_filename').value
        self.enable_motion_gating = self.get_parameter('enable_motion_gating').value
        self.invert_z = self.get_parameter('invert_z').value

        # TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Laser projection utility
        self.laser_projector = LaserProjection()

        # Storage for voxel-filtered accumulated map points
        self.map_points = {}
        self.scan_count = 0

        # Subscribe to 2D scan, publish 3D cloud (individual scans)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.cloud_pub = self.create_publisher(
            PointCloud2, '/pointcloud_3d', 10)

        # Publish the accumulated persistent map
        self.map_pub = self.create_publisher(
            PointCloud2, '/map_3d', 10)

        # Subscribe to motion gating topic
        self.moving_sub = self.create_subscription(
            Bool, '/moving', self.moving_callback, 10)
        self.is_moving = False

        # Define PointCloud2 fields
        self.map_fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        self.get_logger().info('ScanToPointCloud node started')
        self.get_logger().info(f'Projecting scans into frame: {self.target_frame}')
        self.get_logger().info(f'Voxel filter resolution set to: {self.voxel_size}m')

    def moving_callback(self, msg):
        self.is_moving = msg.data

    def scan_callback(self, scan_msg):
        try:
            # 1. Project the LaserScan into a PointCloud2 in its own frame (e.g. 'laser')
            cloud_in_laser_frame = self.laser_projector.projectLaser(scan_msg)

            # 2. Lookup the transform from the laser frame to the target frame
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    scan_msg.header.frame_id,
                    scan_msg.header.stamp,
                    rclpy.duration.Duration(seconds=0.05)
                )
            except Exception:
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    scan_msg.header.frame_id,
                    rclpy.time.Time(),
                    rclpy.duration.Duration(seconds=0.05)
                )

            # 3. Transform the PointCloud2 into the target frame
            cloud_in_target_frame = tf2_sensor_msgs.do_transform_cloud(
                cloud_in_laser_frame, transform)

            # Publish the single real-time scan
            self.cloud_pub.publish(cloud_in_target_frame)

            # 4. Accumulate and voxel-filter points
            should_map = True
            if self.enable_motion_gating and self.is_moving:
                should_map = False
                self.get_logger().info("Mapping paused (Rover is moving)", throttle_duration_sec=5.0)

            if should_map:
                points = pc2.read_points(
                    cloud_in_target_frame,
                    field_names=['x', 'y', 'z', 'intensity'],
                    skip_nans=True
                )
                
                for p in points:
                    x, y, z, intensity = p
                    if self.invert_z:
                        z = -z
                    
                    vx = int(x / self.voxel_size)
                    vy = int(y / self.voxel_size)
                    vz = int(z / self.voxel_size)
                    key = (vx, vy, vz)
                    self.map_points[key] = (float(x), float(y), float(z), float(intensity))

            # 5. Periodically publish the accumulated map (every 10 scans)
            self.scan_count += 1
            if self.scan_count % 10 == 0:
                self.publish_map(cloud_in_target_frame.header)

        except Exception as e:
            self.get_logger().warn(
                f'Could not transform/accumulate scan: {e}', throttle_duration_sec=2.0)

    def publish_map(self, header):
        if not self.map_points:
            return
        try:
            points_list = list(self.map_points.values())
            map_msg = pc2.create_cloud(header, self.map_fields, points_list)
            self.map_pub.publish(map_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish accumulated map: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ScanToPointCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
