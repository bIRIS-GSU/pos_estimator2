# mastermind_node

# This node is responsible for directing the operation of the full system


import roslib
roslib.load_manifest('turtlebot_actions')

import rospy

from turtlebot_actions.msg import *
from actionlib_msgs.msg import *

import actionlib


class MastermindNode():

    def __init__(self):
        # initialize ROS node
        rospy.init_node('mastermind_node')
        
        # Initialize turtlebot_move action client
        self.action_client = action_client = actionlib.SimpleActionClient('turtlebot_move', TurtlebotMoveAction)
        self.action_client.wait_for_server() # Wait for the server to come up
        
        self.srv_acquire_train_data = rospy.ServiceProxy('acquire_train_data', AcquireTrainData)
        self.srv_get_current_position = rospy.ServiceProxy('get_current_position', GetPosition)
        self.srv_correct_errors = rospy.ServiceProxy('correct_errors', CorrectErrors)
        self.srv_train_cnns = rospy.ServiceProxy('train_cnns', TrainCNNs)
        
        # Wait for services to start up
        rospy.loginfo('Waiting for necessary services to start')
        self.rospy.wait_for_service('acquire_train_data')
        self.rospy.wait_for_service('get_current_position')
        self.rospy.wait_for_service('correct_errors')
        self.rospy.wait_for_service('train_cnns')
        
        # That concludes setup, now we need to kick things off
        
        

    def goto_goal(self, x, theta):
        """  Instructs the robot to move to a new, relative position 
        
        Note: The turtlebot_actions process to move is to turn to meet
            theta radians, then move foward x meters."""
        
        action_goal = TurtlebotMoveGoal()
        action_goal.turn_distance = theta
        action_goal.forward_distance = x
        
        if not action_client.send_goal_and_wait(action_goal, rospy.Duration(50.0) == GoalStatus.SUCCEEDED:
            rospy.loginfo('Call to turtlebot_actions server failed')

    def acquire_train_data(self):
        """ Calls the service to acquire training data from the node responsible for that task"""
        try:
            resp = self.srv_acquire_train_data()
        except rospy.ServiceException as exc:
            rospy.loginfo('Service \'acquire_train_data\' could not process request: ' + str(exc))
            
        # After acquiring training data for this positional node, we move to the next position
        

    def get_current_position(self):
        """ Calls the service to get the current position """
        try:
            resp = self.srv_get_current_position()
        except rospy.ServiceException as exc:
            rospy.loginfo('Service \'get_current_position\' could not process request: ' + str(exc))
            
        # Everytime the mastermind asks for current position, we follow by correcting errors
        correct_errors()

    def correct_errors(self):
        """ Calls the service to distribute error corrections over already-gathered data """
        try:
            resp = self.srv_correct_errors()
        except rospy.ServiceException as exc:
            rospy.loginfo('Service \'correct_errors\' could not process request: ' + str(exc))
            
        # After correcting errors, we always retrain the system
        train_cnns()

    def train_cnns(self):
        """ Calls the service to train/retrain the CNNs """ 
        try:
            resp = self.srv_train_cnns()
        except rospy.ServiceException as exc:
            rospy.loginfo('Service \'train_cnns\' could not process request: ' + str(exc))
            
        # After training/retraining the CNNs, we resume with movement again


if __name__ == "__main__":
    try:
	    MastermindNode()
	except rospy.ROSInterruptException:
	    pass