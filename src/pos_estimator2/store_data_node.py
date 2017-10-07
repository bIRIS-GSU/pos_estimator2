# store_data_node

# This node is responsible for storing data in folder names 
# corresponding to the current pose coordinates.


def store_data_node():
    """ Stores training data in a directory somewhere """

    def __init__(self):
        
        rospy.init_node('store_data_node')

        # Setup service
        self.srv_store_data = rospy.Service('store_train_data', StoreTrainData, self.store_train_data)
        
    def store_train_data(self):


if __name__ == "__main__":
    try:
	    store_data_node()
	except rospy.ROSInterruptException:
	    pass