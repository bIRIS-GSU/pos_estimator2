# store_data_node

# This node is responsible for storing data in folder names 
# corresponding to the current pose coordinates.

import rospy
import time

from cv_bridge import CvBridge, CvBridgeError

import cv2

# Import our stuff
import StoreTrainData, RGBDepthImage


def store_data_node():
    """ Stores training data in a directory somewhere """

    def __init__(self):
        
        rospy.init_node('store_data_node')
        
        self.results_dir = rospy.get_param('data_dir')
        if not self.data_dir.endswith('/'):
            rospy.signal_shutdown('Parameter \'data_dir\' should end with character \/')
        
        bridge = CvBridge()

        # Set up service
        self.srv_store_data = rospy.Service('store_train_data', StoreTrainData, self.store_train_data)
        
    def store_train_data(self, msg):
        """ Stores msg.combined_image.rgb and msg.combined_image.depth in a folder named from msg.position """
        
        try:
            cv2_rgb = bridge.imgmsg_to_cv2(msg.combined_image.rgb, "bgr8")
            cv2_depth = bridge.imgmsg_to_cv2(msg.combined_image.depth, "bgr8")
        except CvBridgeError, e:
            rospy.loginfo(e)
        else:
            # Save the files
            pos = msg.position
            pos_dir_name = str(pos.x) + '-' + str(pos.y) + '-' + str(pos.theta) 
            rgb_path = self.data_dir + pos_dir_name + '/rgb/'
            depth_path = self.data_dir + pos_dir_name + '/depth/'
            
            cv2.imwrite(rgb_path + rospy.Time.to_nsec() + '.jpeg', cv2_rgb)
            cv2.imwrite(depth_path + rospy.Time.to_nsec() + '.jpeg', cv2_depth)


if __name__ == "__main__":
    try:
	    store_data_node()
	except rospy.ROSInterruptException:
	    pass