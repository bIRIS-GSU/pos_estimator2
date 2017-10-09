# error_correction_node

# This node calculates the necessary error correction and renames data folders
#   in order to reclassify data for retraining.


class ErrorCorrection():
    """ Corrects errors by acquiring the evaluated position, calculating the error, 
        and distributing the error over multiple training locations """
        
    def __init__(self):
        rospy.init_node('error_correction')
        
        # Setup services        
        self.srv_correct_errors = rospy.Service('correct_errors', CorrectErrors, correct_errors)
        
    def correct_errors(pos):
        """ ROS service that calculates errors and renames folders based on the given current position 
        
        Args:
          pos: geometry_msgs/Pose2D corresponding to current position
          
        Returns:
          None
        """
        
        # Calculate error (if reference is (0,0,0) then error should just be the value of our current position
        # Except for theta, which could be different (by a multiple of 90 deg)
        
        # Figure out which folders need to have their error corrected
        
        # Rename those folders
        


if __name__ == "__main__":
    try:
	    ErrorCorrection()
	except rospy.ROSInterruptException:
	    pass