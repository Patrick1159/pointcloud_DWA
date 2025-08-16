#!/usr/bin/env python

import rospy
import numpy as np
import tf
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Point
import struct
import math

class FakeObstaclePublisher:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('fake_obstacle_publisher', anonymous=True)
        
        # Parameters
        self.static_obstacle_count = rospy.get_param('~static_obstacle_count', 5)
        self.dynamic_obstacle_count = rospy.get_param('~dynamic_obstacle_count', 3)
        self.update_rate = 30 # Hz
        self.obstacle_frame = rospy.get_param('~obstacle_frame', 'odom')
        rospy.loginfo("Obstacle frame: %s", self.obstacle_frame)
        self.obstacle_size = rospy.get_param('~obstacle_size', 0.3)  # meters
        self.max_x = rospy.get_param('~max_x', 10.0)  # meters
        self.max_y = rospy.get_param('~max_y', 10.0)  # meters
        
        # Setup publishers
        self.static_obstacle_pub = rospy.Publisher('static_obstacles', PointCloud2, queue_size=1)
        self.dynamic_obstacle_pub = rospy.Publisher('dynamic_obstacles', PointCloud2, queue_size=1)
        
        # Setup static obstacles
        self.static_obstacles = self.generate_static_obstacles()
        
        # Setup dynamic obstacles
        self.dynamic_obstacles = self.generate_dynamic_obstacles()
        self.dynamic_obstacle_velocities = []
        for _ in range(self.dynamic_obstacle_count):
            # Random velocity between -0.5 and 0.5 m/s for x and y
            vx = (np.random.random() - 0.5) * 1.0
            vy = (np.random.random() - 0.5) * 1.0
            self.dynamic_obstacle_velocities.append((vx, vy))
        
        # Set up timer for publishing
        self.timer = rospy.Timer(rospy.Duration(1.0/self.update_rate), self.timer_callback)
        
        rospy.loginfo("Fake obstacle publisher initialized")
    
    def generate_static_obstacles(self):
        """Generate random static obstacles with different 3D solid shapes"""
        obstacles = []
        
        # shape_types = ['cylinder', 'box', 'cone']
        shape_types = ['box']
        
        for i in range(self.static_obstacle_count):
            # Randomly choose a shape type
            shape_type = np.random.choice(shape_types)
            
            # Generate center position
            center_x = (np.random.random() * self.max_x * 0.8) - (self.max_x * 0.4)
            center_y = (np.random.random() * self.max_y * 0.8) - (self.max_y * 0.4)
            
            # Generate random height for all shapes
            height = 0.5 + np.random.random() * 1.0  # Random height between 0.5 and 1.5 meters
            
            # Generate shape points based on type
            if shape_type == 'cylinder':
                # Generate points forming a solid cylinder
                radius = 0.3 + np.random.random() * 0.5  # Random radius between 0.3 and 0.8 meters
                
                # Number of points in radius and height directions
                radial_samples = 8
                angular_samples = 12
                height_samples = 5
                
                # Generate points throughout the volume
                for h_idx in range(height_samples):
                    z = (h_idx / (height_samples - 1)) * height
                    
                    # Include some interior points at each height level
                    for r_idx in range(radial_samples):
                        # Scale radius from center (r=0) to edge (r=radius)
                        r = (r_idx / (radial_samples - 1)) * radius
                        
                        for angle_idx in range(angular_samples):
                            angle = (angle_idx / angular_samples) * 2 * np.pi
                            x = center_x + r * np.cos(angle)
                            y = center_y + r * np.sin(angle)
                            obstacles.append((x, y, z))
                    
            elif shape_type == 'box':
                # Generate points forming a solid box
                width = 0.3 + np.random.random() * 0.7  # Random width between 0.3 and 1.0 meters
                length = 0.3 + np.random.random() * 0.7  # Random length between 0.3 and 1.0 meters
                
                # Generate a rotation angle for the box
                theta = np.random.random() * 2 * np.pi
                
                # Number of points in each dimension
                width_samples = 2
                length_samples = 2
                height_samples = 2
                
                # Generate points throughout the volume
                for w_idx in range(width_samples):
                    w = ((w_idx / (width_samples - 1)) - 0.5) * width
                    
                    for l_idx in range(length_samples):
                        l = ((l_idx / (length_samples - 1)) - 0.5) * length
                        
                        for h_idx in range(height_samples):
                            z = (h_idx / (height_samples - 1)) * height
                            
                            # Rotate the x-y position
                            rotated_x = w * np.cos(theta) - l * np.sin(theta)
                            rotated_y = w * np.sin(theta) + l * np.cos(theta)
                            
                            # Translate to center
                            x = center_x + rotated_x
                            y = center_y + rotated_y
                            
                            obstacles.append((x, y, z))
                    
            else:  # Cone
                # Generate points forming a solid cone
                base_radius = 0.4 + np.random.random() * 0.6  # Random radius between 0.4 and 1.0 meters
                
                # Number of points in radius and height directions
                radial_samples = 8
                angular_samples = 12
                height_samples = 5
                
                # Generate points throughout the volume
                for h_idx in range(height_samples):
                    z = (h_idx / (height_samples - 1)) * height
                    # Radius decreases linearly with height
                    level_radius = base_radius * (1 - (h_idx / (height_samples - 1)))
                    
                    # Include interior points
                    for r_idx in range(radial_samples):
                        # Scale radius from center (r=0) to edge (r=level_radius)
                        r = (r_idx / (radial_samples - 1)) * level_radius
                        
                        for angle_idx in range(angular_samples):
                            if r > 0 or angle_idx == 0:  # Only include one point at the center
                                angle = (angle_idx / angular_samples) * 2 * np.pi
                                x = center_x + r * np.cos(angle)
                                y = center_y + r * np.sin(angle)
                                obstacles.append((x, y, z))
        
        return obstacles

    def generate_dynamic_obstacles(self):
        """Generate random dynamic obstacles with different 3D solid shapes"""
        obstacles = []
        obstacle_data = []
        
        shape_types = ['cylinder', 'box']
        
        for i in range(self.dynamic_obstacle_count):
            # Randomly choose a shape type
            shape_type = np.random.choice(shape_types)
            
            # Generate center position
            center_x = (np.random.random() * self.max_x * 0.8) - (self.max_x * 0.4)
            center_y = (np.random.random() * self.max_y * 0.8) - (self.max_y * 0.4)
            
            # Generate random height for all shapes
            height = 0.4 + np.random.random() * 0.8  # Random height between 0.4 and 1.2 meters
            
            # Store shape info for this obstacle
            shape_info = {
                'type': shape_type,
                'center': [center_x, center_y, height/2],  # Store center with z being half height
                'points': [],
                'params': {'height': height}
            }
            
            # Generate shape points based on type
            if shape_type == 'cylinder':
                # Generate points forming a cylinder
                radius = 0.2 + np.random.random() * 0.4  # Random radius between 0.2 and 0.6 meters
                shape_info['params']['radius'] = radius
                
                # Number of points in radius and height directions
                radial_samples = 6
                angular_samples = 8
                height_samples = 4
                
                # Generate relative points (will be transformed in update function)
                for h_idx in range(height_samples):
                    z = (h_idx / (height_samples - 1)) * height - height/2  # Center around z=0
                    
                    for r_idx in range(radial_samples):
                        r = (r_idx / (radial_samples - 1)) * radius
                        
                        for angle_idx in range(angular_samples):
                            angle = (angle_idx / angular_samples) * 2 * np.pi
                            rel_x = r * np.cos(angle)
                            rel_y = r * np.sin(angle)
                            rel_z = z
                            
                            shape_info['points'].append((rel_x, rel_y, rel_z))
                            
                            # Initial position
                            x = center_x + rel_x
                            y = center_y + rel_y
                            z = height/2 + rel_z  # Adjust to sit on ground
                            obstacles.append([x, y, z])
                    
            elif shape_type == 'box':
                # Generate points forming a box
                width = 0.3 + np.random.random() * 0.5  # Random width between 0.3 and 0.8 meters
                length = 0.2 + np.random.random() * 0.4  # Random length between 0.2 and 0.6 meters
                theta = np.random.random() * 2 * np.pi  # Random rotation
                
                shape_info['params']['width'] = width
                shape_info['params']['length'] = length
                shape_info['params']['theta'] = theta
                
                # Number of points in each dimension
                width_samples = 10
                length_samples = 10
                height_samples = 10
                
                # Generate relative points (will be transformed in update function)
                for w_idx in range(width_samples):
                    w = ((w_idx / (width_samples - 1)) - 0.5) * width
                    
                    for l_idx in range(length_samples):
                        l = ((l_idx / (length_samples - 1)) - 0.5) * length
                        
                        for h_idx in range(height_samples):
                            z = (h_idx / (height_samples - 1)) * height - height/2  # Center around z=0
                            
                            # Rotate in x-y plane
                            rotated_x = w * np.cos(theta) - l * np.sin(theta)
                            rotated_y = w * np.sin(theta) + l * np.cos(theta)
                            
                            shape_info['points'].append((rotated_x, rotated_y, z))
                            
                            # Initial position
                            x = center_x + rotated_x
                            y = center_y + rotated_y
                            z = height/2 + z  # Adjust to sit on ground
                            obstacles.append([x, y, z])
            
            obstacle_data.append(shape_info)
        
        self.dynamic_obstacle_data = obstacle_data
        return obstacles

    def update_dynamic_obstacles(self):
        """Update positions of dynamic obstacles based on their velocities"""
        dt = 1.0 / self.update_rate
        obstacles = []
        
        for i, obstacle_info in enumerate(self.dynamic_obstacle_data):
            center_x, center_y, center_z = obstacle_info['center']
            vx, vy = self.dynamic_obstacle_velocities[i]
            
            # Update position
            center_x += vx * dt
            center_y += vy * dt
            
            # Bounce off boundaries
            if abs(center_x) > self.max_x / 2:
                vx = -vx
                center_x = np.sign(center_x) * self.max_x / 2
            
            if abs(center_y) > self.max_y / 2:
                vy = -vy
                center_y = np.sign(center_y) * self.max_y / 2
            
            # Update stored position and velocity
            obstacle_info['center'] = [center_x, center_y, center_z]
            self.dynamic_obstacle_velocities[i] = (vx, vy)
            
            # Generate the obstacle points based on shape info
            shape_type = obstacle_info['type']
            shape_points = obstacle_info['points']
            
            # For all shape types, simply translate the relative points to current position
            for relative_x, relative_y, relative_z in shape_points:
                x = center_x + relative_x
                y = center_y + relative_y
                z = center_z + relative_z
                obstacles.append([x, y, z])
        
        return obstacles

    def create_point_cloud(self, points):
        """Convert list of points to PointCloud2 message"""
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        # Convert list of (x,y,z) tuples to list of [x,y,z] points
        cloud_points = []
        for point in points:
            cloud_points.append([point[0], point[1], point[2]])  # Include z for 3D obstacles
        
        # Create the point cloud message
        header = rospy.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = self.obstacle_frame
        
        return pc2.create_cloud(header, fields, cloud_points)
    
    def timer_callback(self, event):
        """Callback for publishing obstacles"""
        # Update dynamic obstacles positions
        self.dynamic_obstacles = self.update_dynamic_obstacles()
        
        # Publish static obstacles
        static_cloud = self.create_point_cloud(self.static_obstacles)
        self.static_obstacle_pub.publish(static_cloud)
        
        # # Publish dynamic obstacles
        # dynamic_cloud = self.create_point_cloud(self.dynamic_obstacles)
        # self.dynamic_obstacle_pub.publish(dynamic_cloud)

def main():
    try:
        obstacle_publisher = FakeObstaclePublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Fake obstacle publisher terminated")

if __name__ == '__main__':
    main()