#!/usr/bin/env python3
"""
Multimodal Human Detection Fusion Node
======================================
Fuses 3D point cloud clusters from /map_3d with 2D thermal camera YOLOv11 detections
from /thermal_detections. Projects 3D clusters onto the camera sensor plane using
coordinate transformation and calculates a fused human-presence confidence.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from vision_msgs.msg import Detection2DArray, Detection2D
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener
import sensor_msgs_py.point_cloud2 as pc2
import math
import time

def rotate_point(p, q):
    # Rotate 3D point p = (x, y, z) by quaternion q = (x, y, z, w)
    # Using formula: v' = v + 2 * u x (u x v + w * v)
    ux, uy, uz, w = q.x, q.y, q.z, q.w
    vx, vy, vz = p
    
    tx = 2.0 * (uy * vz - uz * vy)
    ty = 2.0 * (uz * vx - ux * vz)
    tz = 2.0 * (ux * vy - uy * vx)
    
    rx = vx + w * tx + (uy * tz - uz * ty)
    ry = vy + w * ty + (uz * tx - ux * tz)
    rz = vz + w * tz + (ux * ty - uy * tx)
    
    return (rx, ry, rz)

class MultimodalFusionNode(Node):
    def __init__(self):
        super().__init__('multimodal_fusion_node')
        
        # Declare parameters
        self.declare_parameter('lidar_frame', 'base_link')
        self.declare_parameter('camera_frame', 'thermal_camera_link')
        self.declare_parameter('image_width', 256)
        self.declare_parameter('image_height', 192)
        self.declare_parameter('focal_length', 275.0)
        self.declare_parameter('thermal_timeout', 3.0)  # Seconds
        self.declare_parameter('grid_resolution', 0.10)  # 10 cm voxel size for clustering
        self.declare_parameter('min_points_in_cluster', 4)
        self.declare_parameter('lidar_only', True)       # Revert to LiDAR-only mode for current testing
        
        self.lidar_frame = self.get_parameter('lidar_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        self.focal_length = self.get_parameter('focal_length').value
        self.thermal_timeout = self.get_parameter('thermal_timeout').value
        self.grid_resolution = self.get_parameter('grid_resolution').value
        self.min_points_in_cluster = self.get_parameter('min_points_in_cluster').value
        self.lidar_only = self.get_parameter('lidar_only').value
        
        # TF2 listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Cache for thermal detections
        self.latest_thermal_detections = []
        self.latest_thermal_stamp = 0.0
        
        # Track active RViz marker IDs to clean up old markers
        self.previous_marker_ids = set()
        
        # Subscriptions
        self.map_sub = self.create_subscription(
            PointCloud2, '/map_3d', self.map_callback, 10)
        self.thermal_sub = self.create_subscription(
            Detection2DArray, '/thermal_detections', self.thermal_callback, 10)
            
        # Publishers
        self.marker_pub = self.create_publisher(
            MarkerArray, '/fused_human_markers', 10)
            
        self.get_logger().info("Multimodal Human Detection Fusion Node started.")
        self.get_logger().info(f"Lidar Frame: {self.lidar_frame} | Camera Frame: {self.camera_frame}")
        self.get_logger().info(f"Thermal Resolution: {self.image_width}x{self.image_height}")

    def thermal_callback(self, msg):
        self.latest_thermal_detections = msg.detections
        # Save timestamp as float seconds
        self.latest_thermal_stamp = self.get_clock().now().nanoseconds / 1e9

    def map_callback(self, msg):
        # 1. Check if thermal detections are fresh
        now_sec = self.get_clock().now().nanoseconds / 1e9
        thermal_is_fresh = (now_sec - self.latest_thermal_stamp) < self.thermal_timeout
        
        # 2. Extract and dynamically downsample points using numpy
        try:
            np_points = pc2.read_points_numpy(msg, field_names=['x', 'y', 'z'], skip_nans=True)
        except Exception as e:
            self.get_logger().error(f"Failed to read points using numpy: {e}")
            return
            
        num_points = len(np_points)
        if num_points == 0:
            return
            
        # Target max 5000 points to keep clustering execution under 50ms in Python
        if num_points > 5000:
            step = int(num_points / 5000)
            downsampled_points = np_points[::step]
        else:
            downsampled_points = np_points
            
        # Voxel grid downsampling
        grid = {}
        for p in downsampled_points:
            x, y, z = float(p[0]), float(p[1]), float(p[2])
            gx = int(x / self.grid_resolution)
            gy = int(y / self.grid_resolution)
            gz = int(z / self.grid_resolution)
            key = (gx, gy, gz)
            if key not in grid:
                grid[key] = []
            grid[key].append((x, y, z))
            
        # BFS clustering on voxel grid keys
        unvisited = set(grid.keys())
        clusters = []
        
        while unvisited:
            key = unvisited.pop()
            queue = [key]
            cluster_keys = [key]
            
            while queue:
                curr = queue.pop(0)
                cx, cy, cz = curr
                for dx in [-1, 0, 1]:
                    for dy in [-1, 0, 1]:
                        for dz in [-1, 0, 1]:
                            if dx == 0 and dy == 0 and dz == 0:
                                continue
                            neighbor = (cx + dx, cy + dy, cz + dz)
                            if neighbor in unvisited:
                                unvisited.remove(neighbor)
                                queue.append(neighbor)
                                cluster_keys.append(neighbor)
                                
            cluster_points = []
            for k in cluster_keys:
                cluster_points.extend(grid[k])
                
            if len(cluster_points) >= self.min_points_in_cluster:
                clusters.append(cluster_points)
                
        # 3. Analyze geometry of clusters (Height / Width criteria)
        human_candidates = []
        for idx, cluster in enumerate(clusters):
            xs = [p[0] for p in cluster]
            ys = [p[1] for p in cluster]
            zs = [p[2] for p in cluster]
            
            min_x, max_x = min(xs), max(xs)
            min_y, max_y = min(ys), max(ys)
            min_z, max_z = min(zs), max(zs)
            
            dx = max_x - min_x
            dy = max_y - min_y
            dz = max_z - min_z
            
            centroid_x = sum(xs) / len(xs)
            centroid_y = sum(ys) / len(ys)
            centroid_z = sum(zs) / len(zs)
            
            # SAR Human dimensions check: 
            # standing, sitting or lying human footprint size is usually between 20cm and 1.2m
            # height is between 30cm and 2.0m
            if 0.2 <= dx <= 1.2 and 0.2 <= dy <= 1.2 and 0.3 <= dz <= 2.0:
                c_3d = 0.55  # Base Lidar geometric confidence
                
                # Boost confidence if the height-to-width ratio matches a human frame
                ratio = dz / max(dx, dy)
                if 1.5 <= ratio <= 4.0:
                    c_3d += 0.15
                    
                human_candidates.append({
                    'centroid': (centroid_x, centroid_y, centroid_z),
                    'bbox': (min_x, max_x, min_y, max_y, min_z, max_z),
                    'c_3d': c_3d,
                    'size': (dx, dy, dz)
                })
                
        # 4. Lookup TF Transform for 3D->2D projection
        has_tf = False
        t_vec = None
        q_vec = None
        try:
            transform = self.tf_buffer.lookup_transform(
                self.camera_frame,
                msg.header.frame_id,
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.05)
            )
            t_vec = transform.transform.translation
            q_vec = transform.transform.rotation
            has_tf = True
        except Exception:
            pass # Fallback will be triggered dynamically inside project_3d_to_2d

        # 5. Project Candidates and Match with Thermal Bounding Boxes
        fused_humans = []
        
        for candidate in human_candidates:
            c_3d = candidate['c_3d']
            centroid = candidate['centroid']
            
            # Project centroid to image coordinates
            img_pt = self.project_3d_to_2d(centroid, has_tf, t_vec, q_vec)
            
            matched_thermal_conf = 0.0
            if img_pt is not None and thermal_is_fresh:
                u, v = img_pt
                # Find matching 2D detection containing (u, v)
                for det in self.latest_thermal_detections:
                    bbox = det.bbox
                    left = bbox.center.position.x - bbox.size_x / 2.0
                    right = bbox.center.position.x + bbox.size_x / 2.0
                    top = bbox.center.position.y - bbox.size_y / 2.0
                    bottom = bbox.center.position.y + bbox.size_y / 2.0
                    
                    if left <= u <= right and top <= v <= bottom:
                        # Match found! Get maximum score from results
                        score = max([res.hypothesis.score for res in det.results]) if det.results else 0.0
                        if score > matched_thermal_conf:
                            matched_thermal_conf = score
                            
            # Confidence Fusion
            if self.lidar_only:
                fused_conf = c_3d
                source_str = "Lidar Only"
            elif matched_thermal_conf > 0.0:
                # Combined sensor probability: P(A or B) = 1 - (1-P(A))*(1-P(B))
                fused_conf = 1.0 - (1.0 - matched_thermal_conf) * (1.0 - c_3d)
                source_str = "Fused (Lidar+Thermal)"
            else:
                # No thermal match: penalize geometric confidence
                fused_conf = c_3d * 0.5
                source_str = "Lidar Only"
                
            fused_humans.append({
                'centroid': centroid,
                'size': candidate['size'],
                'confidence': fused_conf,
                'source': source_str
            })
            
        # 6. Publish RViz Markers
        self.publish_markers(msg.header, fused_humans)

    def project_3d_to_2d(self, p_3d, has_tf, t=None, q=None):
        if has_tf:
            rx, ry, rz = rotate_point(p_3d, q)
            x_c = rx + t.x
            y_c = ry + t.y
            z_c = rz + t.z
            
            # If camera frame is physical, convert it to standard optical frame (z-forward, x-right, y-down)
            if 'optical' not in self.camera_frame.lower():
                x_opt = -y_c
                y_opt = -z_c
                z_opt = x_c
            else:
                x_opt = x_c
                y_opt = y_c
                z_opt = z_c
        else:
            # Fallback: Assume camera is mounted 10cm forward, 15cm up, looking straight ahead
            x, y, z = p_3d
            dx, dy, dz = 0.10, 0.0, 0.15
            # Convert robot frame (x-forward, y-left, z-up) to optical camera frame (z-forward, x-right, y-down)
            x_opt = -(y - dy)
            y_opt = -(z - dz)
            z_opt = x - dx
            
        if z_opt <= 0.1: # Must be in front of the camera
            return None
            
        # Standard camera projection matrix
        u = (self.focal_length * x_opt) / z_opt + (self.image_width / 2.0)
        v = (self.focal_length * y_opt) / z_opt + (self.image_height / 2.0)
        
        return (u, v)

    def publish_markers(self, header, fused_humans):
        marker_array = MarkerArray()
        current_marker_ids = set()
        
        for idx, human in enumerate(fused_humans):
            centroid = human['centroid']
            size = human['size']
            conf = human['confidence']
            source = human['source']
            
            # Marker ID mapping
            sphere_id = idx * 2
            text_id = idx * 2 + 1
            
            # 1. 3D Sphere Marker
            marker = Marker()
            marker.header = header
            marker.ns = "fused_humans"
            marker.id = sphere_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(centroid[0])
            marker.pose.position.y = float(centroid[1])
            marker.pose.position.z = float(centroid[2])
            
            # Set scale (approx human size or generic marker size)
            marker.scale.x = float(size[0])
            marker.scale.y = float(size[1])
            marker.scale.z = float(size[2])
            
            # Set color based on confidence (Green = High, Yellow = Medium, Red = Low)
            # If in lidar_only mode, we paint it solid red as requested
            if self.lidar_only:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            else:
                if conf >= 0.7:
                    marker.color.r = 0.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                elif conf >= 0.4:
                    marker.color.r = 1.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                else:
                    marker.color.r = 1.0
                    marker.color.g = 0.0
                    marker.color.b = 0.0
            marker.color.a = 0.6
            
            marker.lifetime = rclpy.duration.Duration(seconds=4.0).to_msg()
            marker_array.markers.append(marker)
            current_marker_ids.add(sphere_id)
            
            # 2. Text Label Marker
            text_marker = Marker()
            text_marker.header = header
            text_marker.ns = "fused_humans"
            text_marker.id = text_id
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = float(centroid[0])
            text_marker.pose.position.y = float(centroid[1])
            text_marker.pose.position.z = float(centroid[2]) + (float(size[2]) / 2.0) + 0.3
            
            text_marker.scale.z = 0.22 # Text height in meters
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            
            text_marker.text = f"{source}: {int(conf * 100)}%"
            text_marker.lifetime = rclpy.duration.Duration(seconds=4.0).to_msg()
            marker_array.markers.append(text_marker)
            current_marker_ids.add(text_id)
            
            self.get_logger().info(
                f"Detected Human at x={centroid[0]:.2f}, y={centroid[1]:.2f}, z={centroid[2]:.2f} "
                f"with confidence: {conf*100:.1f}% ({source})",
                throttle_duration_sec=3.0
            )
            
        # Delete old markers that are no longer active
        for old_id in self.previous_marker_ids:
            if old_id not in current_marker_ids:
                delete_marker = Marker()
                delete_marker.header = header
                delete_marker.ns = "fused_humans"
                delete_marker.id = old_id
                delete_marker.action = Marker.DELETE
                marker_array.markers.append(delete_marker)
                
        self.previous_marker_ids = current_marker_ids
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = MultimodalFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
