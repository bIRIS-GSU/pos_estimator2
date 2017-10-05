# acquire_data_node

# This node is responsible for acquiring odometry, rgb, and depth data


def acquire_data_node():



if __name__ == "__main__":
    try:
	    acquire_data_node()
	except rospy.ROSInterruptException:
	    pass