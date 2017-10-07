# acquire_data_node

# This node is responsible for acquiring odometry, rgb, and depth data

import roslib
roslib.load_manifest('turtlebot_actions')

import rospy
import math

from turtlebot_actions.msg import *
from actionlib_msgs.msg import *

import actionlib

from sensor_msgs.msg import Image


class AcquireDataNode():

    def __init__(self):
    
        ODOM_ONLY = True
    
        # Initialize the ROS node
        rospy.init_node('acquire_data_node')
        
        # Initialize turtlebot_move action client
        self.action_client = action_client = actionlib.SimpleActionClient('turtlebot_move', TurtlebotMoveAction)
        self.action_client.wait_for_server() # Wait for the server to come up
        
        # Setup services
        self.srv_get_position = rospy.ServiceProxy('get_current_position', GetPosition)
        self.srv_store_train_data = rospy.ServiceProxy('store_train_data', StoreTrainData)
        self.srv_acquire_data = rospy.Service('acquire_data', pos_estimator2.srv.AcquireData, acquire_data)
        
        rospy.loginfo('Waiting for necessary services to start')
        rospy.wait_for_service('get_current_position')
        rospy.wait_for_service('store_train_data')
        
    def acquire_data(self):
        """ When triggered, acquires 50 data points, rotates 90 deg, and repeats until 360 deg of rotation """
        
        for i in range(4):
            # Grab current position for storage purposes
            cur_pos = self.srv_get_position(ODOM_ONLY)
            # Acquire 50 data points
            for j in range(50):
                # Acquire the RGB and Depth Images
                rgb_image = rospy.wait_for_message(rospy.get_param('kinect_rgb_topic'), Image)
                depth_image = rospy.wait_for_message(rospy.get_param('kinect_depth_topic'), Image)
                
                # Combine them in our custom message
                combined_image = RGBDepthImage()
                combined_image.rgb = rgb_image
                combined_image.depth = depth_image
                
                # Send the message off to the store_train_data node
                self.srv_store_train_data(combined_image, cur_pos)
                # End for j
            # Spin 90 deg
            spin_turtlebot()
			
		return
        
    def spin_turtlebot(self):
        # Spin 90 degrees between acquiring data
        action_goal = TurtlebotMoveGoal()
        action_goal.turn_distance = math.pi/2
        action_goal.forward_distance = 0
        
        if not self.action_client.send_goal_and_wait(action_goal, rospy.Duration(50.0) == GoalStatus.SUCCEEDED:
            rospy.loginfo('turtlebot_actions server failed failed to spin 90 deg')
        
    

if __name__ == "__main__":
    try:
	    AcquireDataNode()
	except rospy.ROSInterruptException:
	    pass