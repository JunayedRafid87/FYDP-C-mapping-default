#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose

class MockThermalDetector(Node):
    def __init__(self):
        super().__init__('mock_thermal_detector')
        self.pub = self.create_publisher(Detection2DArray, '/thermal_detections', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info("Mock Thermal Detector Node started. Publishing to /thermal_detections...")

    def timer_callback(self):
        msg = Detection2DArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'thermal_camera_link' # Simulated camera frame

        detection = Detection2D()
        detection.header = msg.header
        
        # Define mock bounding box (center_x, center_y, size_x, size_y)
        # Image coordinates (e.g. thermal resolution 256x192)
        # Assume human detected in the middle of the image
        detection.bbox.center.position.x = 128.0  # Middle
        detection.bbox.center.position.y = 96.0   # Middle
        detection.bbox.size_x = 60.0
        detection.bbox.size_y = 120.0

        # Define class and score (confidence)
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = "human"
        hypothesis.hypothesis.score = 0.85 # 85% confidence
        
        detection.results.append(hypothesis)
        msg.detections.append(detection)
        
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MockThermalDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
