# pos_evaluation_node

# This node evaluates our current position based initially on odometry, later
#  a combination of odometry, an RGB CNN, and a depth CNN

import rospy

from sensor_msgs import Odometry, Image
from geometry_msgs import Pose2D




class PosEvaluation():
    """ Evaluates the current position of the robot via odometry and CNNs """
    def __init__(self):
        
        rospy.init_node('pos_evaluation_node')

        # Setup service
        self.srv_evaluate_position = rospy.Service('get_current_position', pos_estimator2.srv.EstimatePosition, get_current_position)
    
    
    def get_current_position():
    
        # Get odometry-based position
        odom = rospy.wait_for_message(rospy.get_param('odometry_topic', Odometry)
        # Calculate euler angles from the odometry quaternion
        euler = tf.transformations.euler_from_quaternion(odom.pose.quaternion)
        # Copy x and y from the odometry pose into our Pose2D, along with theta
        odom_position = Pose2D()
        odom_position.x = odom.pose.pose.position.x
        odom_position.y = odom.pose.pose.position.y
        odom_position.theta = euler[2]
        
        # If the RGB and Depth CNN exist, evaluate position from them as well
        
        # Combine the three estimated positions into our best guess
        
        # Return that position
        current_position = CurrentPosition()
        # Fill current_position much like odom_position was filled earlier
        return current_position


if __name__ == "__main__":
    try:
	    PosEvaluation()
	except rospy.ROSInterruptException:
	    pass