import rospy
import math
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

#!/usr/bin/env python3


def generate_circular_path(center_x, center_y, radius, num_points):
    """Generate a circular path with given center and radius"""
    path = Path()
    path.header.frame_id = "map"
    
    for i in range(num_points):
        angle = 2 * math.pi * i / num_points
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = center_x + radius * math.cos(angle)
        pose.pose.position.y = center_y + radius * math.sin(angle)
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0  # Default orientation (no rotation)
        path.poses.append(pose)
    
    return path

def generate_line_path(start_x, start_y, end_x, end_y, num_points):
    """Generate a straight line path from start to end"""
    path = Path()
    path.header.frame_id = "map"
    
    for i in range(num_points):
        t = float(i) / (num_points - 1)
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = start_x + t * (end_x - start_x)
        pose.pose.position.y = start_y + t * (end_y - start_y)
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)
    
    return path

def generate_goalpoint(end_x, end_y):
    """Generate a goal point at the end of the path"""
    goal = Path()
    goal.header.frame_id = "map"
    goal.header.stamp = rospy.Time.now()

    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = end_x
    pose.pose.position.y = end_y
    pose.pose.position.z = 0.0
    pose.pose.orientation.w = 1.0  # Default orientation (no rotation)
    goal.poses.append(pose)
    return goal

if __name__ == "__main__":
    rospy.init_node("fake_global_path_publisher")
    
    path_pub = rospy.Publisher("/global_path", Path, queue_size=1)
    
    # Parameters
    rate = rospy.get_param("~rate", 60)  # Hz
    path_type = rospy.get_param("~path_type", "goalpoint")  # "line" "circle" "goalpoint"
    
    if path_type == "circle":
        center_x = rospy.get_param("~center_x", 0.0)
        center_y = rospy.get_param("~center_y", 0.0)
        radius = rospy.get_param("~radius", 5.0)
        num_points = rospy.get_param("~num_points", 20)
        path = generate_circular_path(center_x, center_y, radius, num_points)
    elif path_type == "line":  # line
        start_x = rospy.get_param("~start_x", 0.0)
        start_y = rospy.get_param("~start_y", 0.0)
        end_x = rospy.get_param("~end_x", 10.0)
        end_y = rospy.get_param("~end_y", 10.0)
        num_points = rospy.get_param("~num_points", 20)
        path = generate_line_path(start_x, start_y, end_x, end_y, num_points)
    elif path_type == "goalpoint":
        path = generate_goalpoint(
            rospy.get_param("~goal_x", 10.0),
            rospy.get_param("~goal_y", 10.0)
        )
        rospy.loginfo("Publishing goal point only, no path.")
        print(path)
    rate_obj = rospy.Rate(rate)
    
    rospy.loginfo(f"Publishing fake {path_type} global path...")
    
    while not rospy.is_shutdown():
        # Update timestamp
        path.header.stamp = rospy.Time.now()
        for pose in path.poses:
            pose.header.stamp = rospy.Time.now()
        
        path_pub.publish(path)
        rate_obj.sleep()