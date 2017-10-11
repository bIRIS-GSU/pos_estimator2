# mastermind_node

# This node is responsible for directing the operation of the full system


import roslib
roslib.load_manifest('turtlebot_actions')

import rospy



import actionlib


class MastermindNode():
    """ ROS node that coordinates the operation of the Position Estimator """

    def __init__(self):
        # initialize ROS node
        rospy.init_node('mastermind_node')
        
        
        
        self.srv_move_next = rospy.ServiceProxy('move_next', MoveNext)
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
        
        # Get coordinates for movement
        self.coords = rospy.get_param('coords')
        self.num_pts = len(coords)
        self.cur_node = 0 # Start at origin
        
        # That concludes setup
        
        # Main loop of the program
        while 1:
            # Goto a position
            self.goto_next()
            # Now we're at that position
            # If we're at the origin, train, then check and correct errors
            if AT_ORIGIN():
                # First train the CNNs so we can get an estimated position
                self.train_cnns()
                # Get our estimated position
                cur_pos = self.get_current_position()
                # Correct errors based on that position
                self.correct_errors(cur_pos)
                
            # If we're not at the origin, just acquire data
            else:
                self.acquire_train_data()
                
            # Then repeat the process
                
        
        

    def goto_next(self):
        """  Instructs the robot to move to a new, relative position 
        
        Note: The turtlebot_actions process to move is to turn to meet
            theta radians, then move forward x meters. 
            
        Args:
          None
          
        Returns:
          None
        """
        
        self.cur_node += 1
        
        action_goal = TurtlebotMoveGoal()
        action_goal.turn_distance = theta
        action_goal.forward_distance = x
        
        if not self.action_client.send_goal_and_wait(action_goal, rospy.Duration(50.0) == GoalStatus.SUCCEEDED:
            rospy.loginfo('Call to turtlebot_actions server failed')


    def acquire_train_data(self):
        """ Calls ROS service 'acquire_train_data' 
        
        Args:
          None
        
        Returns:
          None
        """
        
        try:
            resp = self.srv_acquire_train_data()
        except rospy.ServiceException as exc:
            rospy.loginfo('Service \'acquire_train_data\' could not process request: ' + str(exc))
            
        

    def get_current_position(self):
        """ Calls ROS service 'get_current_position'

        Args:
          None
          
        Returns:
          geometry_msgs/Pose2D for current position
        """
        try:
            resp = self.srv_get_current_position()
        except rospy.ServiceException as exc:
            rospy.loginfo('Service \'get_current_position\' could not process request: ' + str(exc))
            
        return resp
            

    def correct_errors(self, cur_pos):
        """ Calls Ros service 'correct_errors'

        Args:
          cur_pos: Current position, in the form of a geometry_msgs/Pose2D
          
        Returns:
          None
        """
        
        try:
            resp = self.srv_correct_errors(cur_pos)
        except rospy.ServiceException as exc:
            rospy.loginfo('Service \'correct_errors\' could not process request: ' + str(exc))
            

    def train_cnns(self):
        """ Calls ROS service 'train_cnns'
        
        Args:
          None
          
        Returns:
          None
        """ 
        
        try:
            resp = self.srv_train_cnns()
        except rospy.ServiceException as exc:
            rospy.loginfo('Service \'train_cnns\' could not process request: ' + str(exc))
            


if __name__ == "__main__":
    try:
	    MastermindNode()
	except rospy.ROSInterruptException:
	    pass