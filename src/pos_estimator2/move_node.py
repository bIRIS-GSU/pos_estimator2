# move_node

# This node moves the Turtlebot from point to point

import tf
import math

import rospy

from sensor_messages.msg import Odometry


class MoveNode():
    """ This class moves the Turtlebot from point to point """
    def __init__(self):
    
        self.coords = rospy.get_param('coords')
        tmp = [None]*len(coords)
        for i in coords:
            tmp[int(i)-1] = coords[i]
        self.coords = tmp
        self.num_pts = len(coords)
        self.cur_node = 0 # Start at origin
        
        self.srv_acquire_data = rospy.Service('next_move', NextMove, next_move)
        
    def next_move():
        # Set up empty goal
        action_goal = TurtlebotMoveGoal()
        # Calculate distance to rotate
        cur_theta = get_theta()
        rads = get_radians_to_rotate(cur_theta, self.cur_node)
        
        # Calculate distance to move forward_distance
        dist = get_dist_to_move()
        
        action_goal.turn_distance = rads
        action_goal.forward_distance = dist
        
        if not self.action_client.send_goal_and_wait(action_goal, rospy.Duration(50.0) == GoalStatus.SUCCEEDED:
            rospy.loginfo('Call to turtlebot_actions server failed')
            
        # We got to our goal
        self.cur_node += 1
        if self.cur_node +1 > self.num_pts:
            # This means current node is origin, next node is undefined
            # So calculate the next set of nodes
            self.cur_node = 0
            theta = get_theta()
            self.coords = transform_coords(theta, self.coords)
            # And reset our cur_node to 0
            self.cur_node=0
            
        
    def get_theta():
        # Get odometry-based position
        odom = rospy.wait_for_message(rospy.get_param('odometry_topic', Odometry)
        
        # Calculate euler angles from the odometry quaternion
        euler = tf.transformations.euler_from_quaternion(odom.pose.quaternion)
        
        # Get current theta in radians
        cur_theta = euler[2]
        
    def get_radians_to_rotate(theta, node_num):
        """ Calculates radians to rotate to face next point
        
        Args:
          theta - Current heading in radiance
          node_num - Index of current node in 'coords'
          
        Returns:
          rads - radians to rotate
        """  
         
        if node_num < 1:
            x0,y0 = 0,0
        else:
            x0,y0 = convert_coords(node_num)
        if node_num + 1 > self.num_pts:
            x1,y1 = 0,0
        else:
            x1,y1 = convert_coords(node_num+1)
            
        rads = atan( (y1-y0)/(x1-x0) ) - theta
        return rads
        
    def get_dist_to_move(): 
        """ Calculates distance to move towards next node
        
        Args: 
          None
        
        Returns:
          dist - Distance in meters to move to next node
        """
    
        if node_num < 1:
            x0,y0 = 0,0
        else:
            x0,y0 = convert_coords(node_num)
        if node_num + 1 > self.num_pts:
            x1,y1 = 0,0
        else:
            x1,y1 = convert_coords(node_num+1)
            
        dist = math.sqrt(math.pow(y1-y0,2) + math.pow(x1-x0,2))
        return dist
        
    def convert_coords(coord):
        return float(coord['x']), float(coord['y'])
        
    def transform_coords(coords, theta):
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