#!/usr/bin/env python3
import sys
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2

class PCDPublisher(Node):
    def __init__(self, pcd_filepath, frame_id='base_link', topic='/map_3d'):
        super().__init__('pcd_publisher')
        self.pcd_filepath = pcd_filepath
        self.frame_id = frame_id
        
        self.pub = self.create_publisher(PointCloud2, topic, 10)
        
        # Load the points from PCD and downsample only if they exceed 250,000 points
        raw_points = self.load_pcd(pcd_filepath)
        max_points = 250000
        if len(raw_points) > max_points:
            step = int(len(raw_points) / max_points)
            self.points = raw_points[::step]
            self.get_logger().info(f"Downsampled points from {len(raw_points)} to {len(self.points)} (step size: {step}) to prevent UDP buffer packet drops.")
        else:
            self.points = raw_points
        
        # Define fields
        self.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
        ]
        
        self.get_logger().info(f"Loaded {len(self.points)} points from {pcd_filepath}")
        self.get_logger().info(f"Publishing map to {topic} (frame: {frame_id}) at 0.5 Hz...")
        
        # Publish periodically (every 2 seconds)
        self.timer = self.create_timer(2.0, self.timer_callback)
        
        # Publish immediately once
        self.timer_callback()

    def load_pcd(self, filepath):
        points = []
        if not os.path.exists(filepath):
            self.get_logger().error(f"File not found: {filepath}")
            sys.exit(1)
            
        with open(filepath, 'r') as f:
            lines = f.readlines()
            
        data_idx = -1
        for i, line in enumerate(lines):
            if line.strip().startswith("DATA ascii"):
                data_idx = i + 1
                break
                
        if data_idx == -1:
            self.get_logger().error(f"Only ASCII PCD format is supported. Could not find 'DATA ascii' header in {filepath}")
            sys.exit(1)
            
        for line in lines[data_idx:]:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            if len(parts) >= 3:
                x = float(parts[0])
                y = float(parts[1])
                z = float(parts[2])
                # intensity defaults to 0 if not present in file
                intensity = float(parts[3]) if len(parts) >= 4 else 0.0
                points.append((x, y, z, intensity))
                
        return points

    def timer_callback(self):
        if not self.points:
            return
        
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id
        
        cloud_msg = pc2.create_cloud(header, self.fields, self.points)
        self.pub.publish(cloud_msg)

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 pcd_publisher.py <path_to_pcd_file> [frame_id]")
        sys.exit(1)
        
    pcd_path = sys.argv[1]
    frame_id = sys.argv[2] if len(sys.argv) > 2 else 'base_link'
    
    rclpy.init()
    node = PCDPublisher(pcd_path, frame_id)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
