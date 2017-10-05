# pos_evaluation_node

# This node evaluates our current position based initially on odometry, later
#  a combination of odometry, an RGB CNN, and a depth CNN


def pos_evaluation_node():




if __name__ == "__main__":
    try:
	    pos_evaluation_node()
	except rospy.ROSInterruptException:
	    pass