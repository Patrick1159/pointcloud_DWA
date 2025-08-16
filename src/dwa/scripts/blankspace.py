import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
from struct import pack

#!/usr/bin/env python3

import sensor_msgs.point_cloud2 as pc2
import std_msgs.msg

def publish_empty_pointcloud():
    rospy.init_node('blank_pointcloud_publisher', anonymous=True)
    pub = rospy.Publisher('/blank_pointcloud', PointCloud2, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz
    
    # Create a blank point cloud message
    empty_cloud = PointCloud2()
    empty_cloud.header.frame_id = "odom"  # Frame ID
    
    # Setup fields for PointCloud2
    fields = [
        pc2.PointField(name='x', offset=0, datatype=pc2.PointField.FLOAT32, count=1),
        pc2.PointField(name='y', offset=4, datatype=pc2.PointField.FLOAT32, count=1),
        pc2.PointField(name='z', offset=8, datatype=pc2.PointField.FLOAT32, count=1)
    ]
    
    # No points in the cloud
    points = np.zeros((0, 3), dtype=np.float32)
    
    while not rospy.is_shutdown():
        empty_cloud.header.stamp = rospy.Time.now()
        empty_cloud = pc2.create_cloud(empty_cloud.header, fields, points)
        pub.publish(empty_cloud)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_empty_pointcloud()
    except rospy.ROSInterruptException:
        pass