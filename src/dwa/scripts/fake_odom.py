#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import tf
from tf.transformations import quaternion_from_euler
import math

class FakeOdomNode:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('fake_odom_publisher', anonymous=True)
        
        # Set up publisher for odometry
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=50)
        
        # Set up subscriber for cmd_vel
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
         
        # Initialize transform broadcaster
        self.br = tf.TransformBroadcaster()
        
        # Set the rates
        self.rate = rospy.Rate(30)  # 20 Hz
        
        # Initial position and orientation in odom frame
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        
        # Current velocities (will be updated by cmd_vel)
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0
        
        # Get current time for the first loop
        self.current_time = rospy.Time.now()
        self.last_time = self.current_time

    def cmd_vel_callback(self, msg):
        # Update velocities from cmd_vel
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.vth = msg.angular.z
        rospy.loginfo("Received cmd_vel: vx=%.2f, vy=%.2f, vth=%.2f", self.vx, self.vy, self.vth)

    def update_odometry(self):
        # Calculate time elapsed
        self.current_time = rospy.Time.now()
        dt = (self.current_time - self.last_time).to_sec()
        
        # Debug output of velocities and position
        if dt > 0:  # Avoid division by zero
            pass
            # rospy.loginfo("Velocities: vx=%.2f, vy=%.2f, vth=%.2f", self.vx, self.vy, self.vth)
            # rospy.loginfo("Odometry: x=%.2f, y=%.2f, th=%.2f", self.x, self.y, self.th)
        # Calculate new pose
        delta_x = (self.vx * math.cos(self.th) - self.vy * math.sin(self.th)) * dt
        delta_y = (self.vx * math.sin(self.th) + self.vy * math.cos(self.th)) * dt
        delta_th = self.vth * dt
        
        self.x += delta_x
        self.y += delta_y
        self.th += delta_th
        
        # Create quaternion from yaw
        odom_quat = quaternion_from_euler(0, 0, self.th)
        
        # Create and publish odometry message
        odom = Odometry()
        odom.header.stamp = self.current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        
        # Set the position
        odom.pose.pose = Pose(Point(self.x, self.y, 0.0), Quaternion(*odom_quat))
        
        # Set the velocity
        odom.twist.twist = Twist(Vector3(self.vx, self.vy, 0.0), Vector3(0.0, 0.0, self.vth))
        
        # Publish the message
        self.odom_pub.publish(odom)
        
        # Publish transform
        self.br.sendTransform(
            (self.x, self.y, 0.0),
            odom_quat,
            self.current_time,
            "base_link",
            "odom"
        )
        
        # Publish map->odom transform
        # This is a static transform that can be adjusted as needed
        # In a real system, this would come from a localization system
        map_to_odom_quat = quaternion_from_euler(0, 0, 0)  # No rotation between map and odom
        # self.br.sendTransform(
        #     (0.0, 0.0, 0.0),  # No translation between map and odom frames
        #     map_to_odom_quat,
        #     self.current_time,
        #     "odom",           # Child frame
        #     "map"             # Parent frame
        # )
        
        # Update the last time variable
        self.last_time = self.current_time

    def run(self):
        while not rospy.is_shutdown():
            self.update_odometry()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = FakeOdomNode()
        node.run()
    except rospy.ROSInterruptException:
        pass