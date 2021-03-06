# acquire_data_node

# This node is responsible for acquiring odometry, rgb, and depth data

import roslib
roslib.load_manifest('turtlebot_actions')

import rospy
from math import pi

from turtlebot_actions.msg import *
from actionlib_msgs.msg import *

import actionlib

from sensor_msgs.msg import Image


class AcquireDataNode():
    """ Acquires 50 data points per rotation at the current position. Rotates and acquires data until back at initial heading.
    Rotations are, currently, 90 deg. """

    def __init__(self):
    
        # Initialize the ROS node
        rospy.init_node('acquire_data_node')
        
        # Initialize turtlebot_move action client
        self.action_client = action_client = actionlib.SimpleActionClient('turtlebot_move', TurtlebotMoveAction)
        self.action_client.wait_for_server() # Wait for the server to come up
        
        # Setup services
        self.srv_get_cur_node = rospy.ServiceProxy('get_cur_node', GetCurrentNpde)
        self.srv_store_train_data = rospy.ServiceProxy('store_train_data', StoreTrainData)
        self.srv_acquire_data = rospy.Service('acquire_data', AcquireData, acquire_data)
        
        rospy.loginfo('Waiting for necessary services to start')
        rospy.wait_for_service('get_cur_node')
        rospy.wait_for_service('store_train_data')
        rospy.loginfo('AcquireDataNode initialized!')
        
    def acquire_data(self):
        """ When triggered, acquires 50 data points, rotates 90 deg, and repeats until 360 deg of rotation 
        
        Args: None
        
        Returns: None
        """
        
        for i in range(4):
            # Grab current position for storage purposes
            cur_pos = self.srv_get_cur_node()
            # Acquire 50 data points
            for j in range(50):
                # Acquire the RGB and Depth Images
                rgb_image = rospy.wait_for_message(rospy.get_param('kinect_rgb_topic'), Image)
                depth_image = rospy.wait_for_message(rospy.get_param('kinect_depth_topic'), Image)
                
                # Combine them in our custom message
                combined_image = RGBDepthImage()
                combined_image.rgb = rgb_image
                combined_image.depth = depth_image
                
                # Plug in our theta to the position
                cur_pos.theta = i*pi/2
                # Send the message off to the store_train_data node
                self.srv_store_train_data(combined_image, cur_pos)
                # End for j
            # Spin 90 deg
            spin_turtlebot()
			
		return
        
    def spin_turtlebot(self):
        """ Spins the Turtlebot 90 degrees 
        FUTURE: Add argument or ROS param to specify degrees to rotate
        
        Args: None
        
        Returns: None
        """
        action_goal = TurtlebotMoveGoal()
        action_goal.turn_distance = pi/2
        action_goal.forward_distance = 0
        
        if not self.action_client.send_goal_and_wait(action_goal, rospy.Duration(50.0) == GoalStatus.SUCCEEDED:
            rospy.loginfo('turtlebot_actions server failed failed to spin 90 deg')
        
    

if __name__ == "__main__":
    try:
	    AcquireDataNode()
	except rospy.ROSInterruptException:
	    pass