# pos_evaluation_node

# This node evaluates our current position based initially on odometry, later
#  a combination of odometry, an RGB CNN, and a depth CNN

import rospy

import tf
from sensor_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
from pos_estimator2.srv import EstimatePosition




class PosEvaluation():
    """ Evaluates the current position of the robot via odometry and CNNs """
    def __init__(self):
        
        rospy.init_node('pos_evaluation_node')

        # Setup service
        self.srv_evaluate_position = rospy.Service('get_current_position', EstimatePosition, get_current_position)
    
    
    @property
    def get_current_position(self):
        """ ROS service that evaluates the current position of the robot 
        
        Args:
          None
          
        Returns:
          geometry_msgs/Pose2D current_position
        """
    
        # Get odometry-based position
        odom_pos = self.get_odom_pos()
        
        # If the RGB and Depth CNN exist, evaluate position from them as well
        
        # Combine the three estimated positions into our best guess
        
        # Return that position
        cur_pos = Pose2D()
        # Fill current_position much like odom_position was filled in get_odom_pos()
        return cur_pos
        
    def get_odom_pos(self):
        """ Gets current position based on odometry 
        
        Args:
          None
          
        Returns:
          geometry_msgs/Pose2D current position
        """

        # Note: Odometry also carries a covariance (uncertainty) matrix
        # ... could use this for confidence for odom measurements.
        # Get odometry-based position
        odom = rospy.wait_for_message(rospy.get_param('odometry_topic', Odometry))
        # Calculate euler angles from the odometry quaternion
        euler = tf.transformations.euler_from_quaternion(odom.pose.quaternion)
        # Copy x and y from the odometry pose into our Pose2D, along with theta
        odom_position = Pose2D()
        odom_position.x = odom.pose.pose.position.x
        odom_position.y = odom.pose.pose.position.y
        odom_position.theta = euler[2]
        
        return odom_position
        
    def get_rgb_pos(self):
        """ Gets current position based on RGB CNN
        Args:
          None
          
        Returns:
          geometry_msgs/Pose2D current position
        """

        return Pose2D()
        
    def get_depth_pos(self):
        """ Gets current position based on Depth CNN
        
        Args:
          None
          
        Returns:
          geometry_msgs/Pose2D current position
        """
        return Pose2D()


if __name__ == "__main__":
    try:
        PosEvaluation()
    except rospy.ROSInterruptException:
        pass
