# move_node

# This node moves the Turtlebot from point to point

import tf
import rospy

from sensor_messages.msg import Odometry


class MoveNode():
    """ This class moves the Turtlebot from point to point """
    def __init__(self):
    
        self.coords = rospy.get_param('coords')
        self.num_pts = len(coords)
        self.cur_node = 0 # Start at origin
        
        self.srv_acquire_data = rospy.Service('next_move', NextMove, next_move)
        
    def next_move():
        # Calculate distance to rotate
        cur_theta = get_theta()
        
    def get_theta():
        # Get odometry-based position
        odom = rospy.wait_for_message(rospy.get_param('odometry_topic', Odometry)
        
        # Calculate euler angles from the odometry quaternion
        euler = tf.transformations.euler_from_quaternion(odom.pose.quaternion)
        
        # Get current theta in radians
        cur_theta = euler[2]
    
    
    



if __name__ == "__main__":
    try:
	    MoveNode()
	except rospy.ROSInterruptException:
	    pass