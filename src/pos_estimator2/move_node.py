# move_node

# This node moves the Turtlebot from point to point

# Import ROS related things
import rospy

from turtlebot_actions.msg import TurtlebotMoveAction, TurtlebotMoveGoal
from actionlib_msgs.msg import *

from sensor_messages.msg import Odometry

# Import other modules
import tf
import math




class MoveNode():
    """ This class moves the Turtlebot from point to point """
    def __init__(self):
    
        # Initialize turtlebot_move action client
        self.action_client = actionlib.SimpleActionClient('turtlebot_move', TurtlebotMoveAction)
        self.action_client.wait_for_server() # Wait for the server to come up
    
        self.coords = rospy.get_param('coords')
        tmp = [None]*len(coords)
        for i in coords:
            tmp[int(i)-1] = coords[i]
        self.coords = tmp
        self.num_pts = len(coords)
        self.cur_node = 0 # Start at origin
        
        # Advertise this node's Services
        self.srv_next_move = rospy.Service('next_move', NextMove, next_move)
        self.srv_get_cur_node = rospy.Service('get_cur_node', GetCurrentNode, get_cur_node)
        self.srv_get_nodes = rospy.Service('get_nodes', GetNodes, get_nodes)
        
    def next_move():
        # Set up empty goal
        action_goal = TurtlebotMoveGoal()
        
        # Calculate distance to rotate
        cur_theta = get_theta()
        rads = get_radians_to_rotate(cur_theta, self.cur_node+ 1)
        
        # Calculate distance to move forward_distance
        dist = get_dist_to_move()
        
        action_goal.turn_distance = rads
        action_goal.forward_distance = dist
        
        if not self.action_client.send_goal_and_wait(action_goal, rospy.Duration(50.0) == GoalStatus.SUCCEEDED:
            rospy.loginfo('Call to turtlebot_actions server failed')
            
        # We got to our goal
        # Rotate back to original heading
        cur_theta = get_theta()
        action_goal.turn_distance = -theta
        action_goal.forward_distance = 0
        
        if not self.action_client.send_goal_and_wait(action_goal, rospy.Duration(50.0) == GoalStatus.SUCCEEDED:
            rospy.loginfo('Call to turtlebot_actions server failed')
            
            
        self.cur_node += 1
        if self.cur_node +1 > self.num_pts:
            # This means current node is origin, next node is undefined
            # So calculate next set of nodes
            theta = get_theta()
            self.coords = transform_coords(theta, self.coords)
            # And reset our cur_node to 0
            self.cur_node=0
            
        
    def get_cur_node():
        """ ROS service call that returns Pose2D of current node 
        Args: None
        
        Returns: Pose2D
        """
        
        if self.cur_node == 0 or self.cur_node > self.num_pts:
            return Pose2D(0,0,0)
        else:
            return Pose2D(self.coords[self.cur_node]['x'], self.coords[self.cur_node]['y'], 0)
    
    def get_nodes():
        """ ROS service call that returns Pose2D[] of nodes
        
        Args: None
        
        Returns: Pose2D[] of nodes
        """
        
        nodes = [Pose2D]*coords.num_pts
        for i in nodes:
            nodes[i].x = coords[i]['x']
            nodes[i].y = coords[i]['y']
            nodes[i].theta = 0
        return nodes
    
    def get_theta():
        """ Retrieves heading in radians from ROS topic, param 'odometry_topic'
        
        Args: 
          None
        
        Returns:
          Heading in radians
        """
        
        # Get odometry-based position
        odom = rospy.wait_for_message(rospy.get_param('odometry_topic', Odometry)
        
        # Calculate euler angles from the odometry quaternion
        euler = tf.transformations.euler_from_quaternion(odom.pose.quaternion)
        
        # Get current theta in radians
        cur_theta = euler[2]
        
    def get_radians_to_rotate(theta):
        """ Calculates radians to rotate to face next point
        
        Args:
          theta - Current heading in radiance
          
        Returns:
          rads - radians to rotate
        """  
         
        if self.cur_node < 1:
            x0,y0 = 0,0
        else:
            x0,y0 = convert_coords(self.coords[self.cur_node])
        if self.coords[self.cur_node] + 1 > self.num_pts:
            x1,y1 = 0,0
        else:
            x1,y1 = convert_coords(self.coords[self.cur_node+1])
            
        rads = atan( (y1-y0)/(x1-x0) ) - theta
        return rads
        
    def get_dist_to_move(): 
        """ Calculates distance to move towards next node
        
        Args: 
          None
        
        Returns:
          dist - Distance in meters to move to next node
        """
    
        if self.cur_node < 1:
            x0,y0 = 0,0
        else:
            x0,y0 = convert_coords(self.coords[self.cur_node])
        if self.cur_node + 1 > self.num_pts:
            x1,y1 = 0,0
        else:
            x1,y1 = convert_coords(self.coords[self.cur_node]+1)
            
        dist = math.sqrt(math.pow(y1-y0,2) + math.pow(x1-x0,2))
        return dist
        
    def convert_coords(coord):
        """ Converts the string-based coords from the ROS param to floats 
        
        Args:
          coord - single coordinate dict, with fields 'x' and 'y' corresponding to string values
          
        Returns:
          x,y as float values
        
        """
        return float(coord['x']), float(coord['y'])
        
    def transform_coords(coords, theta):
        """ Rotates provided coords by theta radians
        
        Args:
          coords - array of dicts mapping 'x' and 'y' to float values
          theta - angle in radians to rotate
          
        Returns:
          updated coords array of rotated coordinates
        """
        new_coords = [None]*length(coords)
        for i in coords:
            new_coords[i]['x'] = math.cos(theta)*coords[i]['x'] - math.sin(theta)*coords[i]['y']
            new_coords[i]['y'] = math.sin(theta)*coords[i]['x'] - math.cos(theta)*coords[i]['y']
        return new_coords
        
     


if __name__ == "__main__":
    try:
	    MoveNode()
	except rospy.ROSInterruptException:
	    pass