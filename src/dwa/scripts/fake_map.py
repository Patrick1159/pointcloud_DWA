import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose

#!/usr/bin/env python3


class FakeMapPublisher:
    def __init__(self):
        rospy.init_node('fake_map_publisher')
        
        # Map parameters
        self.width = rospy.get_param('~map_width', 100)  # cells
        self.height = rospy.get_param('~map_height', 100)  # cells
        self.resolution = rospy.get_param('~map_resolution', 0.1)  # m/cell
        self.frame_id = rospy.get_param('~map_frame', 'map')
        self.publish_rate = rospy.get_param('~publish_rate', 1.0)  # Hz
        
        # Create publisher
        self.map_pub = rospy.Publisher('map', OccupancyGrid, queue_size=1, latch=True)
        
        # Timer for publishing
        rospy.Timer(rospy.Duration(1.0/self.publish_rate), self.publish_map)
        
        rospy.loginfo("Fake map publisher initialized")
    
    def create_fake_map(self):
        # Create an empty map
        grid = np.zeros((self.height, self.width), dtype=np.int8)
        
        # Add some obstacles - border walls
        border_width = 3
        grid[:border_width, :] = 100  # Top wall
        grid[-border_width:, :] = 100  # Bottom wall
        grid[:, :border_width] = 100  # Left wall
        grid[:, -border_width:] = 100  # Right wall
        
        # Add some random obstacles
        # num_obstacles = 10
        # for _ in range(num_obstacles):
        #     x = np.random.randint(10, self.width-10)
        #     y = np.random.randint(10, self.height-10)
        #     size = np.random.randint(3, 8)
        #     grid[y-size:y+size, x-size:x+size] = 100
        
        # Add a corridor
        corridor_x = self.width // 2
        corridor_y = self.height // 2
        corridor_width = 5
        grid[corridor_y-corridor_width:corridor_y+corridor_width, :] = 0
        
        return grid.flatten().tolist()
    
    def publish_map(self, event=None):
        # Create map message
        map_msg = OccupancyGrid()
        
        # Fill header
        map_msg.header.stamp = rospy.Time.now()
        map_msg.header.frame_id = self.frame_id
        
        # Fill metadata
        map_msg.info.resolution = self.resolution
        map_msg.info.width = self.width
        map_msg.info.height = self.height
        
        # Set origin (bottom-left corner of the map)
        map_msg.info.origin = Pose()
        map_msg.info.origin.position.x = -self.width * self.resolution / 2.0
        map_msg.info.origin.position.y = -self.height * self.resolution / 2.0
        
        # Fill data
        map_msg.data = self.create_fake_map()
        
        # Publish
        self.map_pub.publish(map_msg)
        rospy.loginfo_throttle(5.0, "Publishing fake map")

if __name__ == '__main__':
    try:
        map_publisher = FakeMapPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass