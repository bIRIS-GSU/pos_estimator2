# error_correction_node

# This node calculates the necessary error correction and renames data folders
#   in order to reclassify data for retraining.


import rospy

import os

class ErrorCorrection():
    """ Corrects errors by acquiring the evaluated position, calculating the error, 
        and distributing the error over multiple training locations """
        
    def __init__(self):
        rospy.init_node('error_correction')
        
        # Setup services        
        self.srv_correct_errors = rospy.Service('correct_errors', CorrectErrors, correct_errors)
        
        self.srv_get_nodes = rospy.ServiceProxy('get_nodes', GetNodes)
        self.srv_get_position = rospy.ServiceProxy('get_current_position', GetPosition)
        
        self.results_dir = rospy.get_param('data_dir')
        
    def correct_errors(pos):
        """ ROS service that calculates errors and renames folders based on the given current position 
        
        Args:
          pos: geometry_msgs/Pose2D corresponding to current position
          
        Returns:
          None
        """
        
        # Calculate error (if reference is (0,0,0) then error should just be the value of our current position
        error = self.srv_get_position()
        # Except for theta, which could be different (by a multiple of 90 deg) (but shouldn't, since we always spin to 0)
        
        # Figure out which nodes we're distributing the error over
        nodes = self.srv_get_nodes()
        
        # Not sure about indexes here... I think it'll be zero-indexed
        # Section distribute the error exponentially, where a given nodes gets twice the error as the previous node
        den = 0
        for i in nodes:
            den += math.pow(2, i)
        distributed_error = [Pose2D]*length(nodes)
        for i in nodes:
            distributed_error[i].x = error.x * math.pow(2, i) / den
            distributed_error[i].y = error.y * math.pow(2, i) / den
            distributed_error[i].theta = error.theta * math.pow(2, i) / den
        
        
        for i in nodes:
            # For each node, calculate old folder name
            old_pos = nodes[i]
            new_pos = distributed_error[i]
            pos_old_dir_name = str(old_pos.x) + '-' + str(old_pos.y) + '-' + str(old_pos.theta) 
            # And then rename it to the new name
            pose_new_dir_name = str(new_pos.x) + '-' + str(new_pos.y) + '-' + str(new_pos.theta) 
            
            os.rename(self.data_dir + pos_old_dir_name, self.data_dir + pos_new_dir_name)
            
        return


if __name__ == "__main__":
    try:
	    ErrorCorrection()
	except rospy.ROSInterruptException:
	    pass