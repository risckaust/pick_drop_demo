#!/usr/bin/env python

# ROS python API
import rospy
import tf

def main:
	# initiate node
    rospy.init_node('tf_listner_test', anonymous=True)

	try:
		(trans,rot) = listener.lookupTransform('/disc', '/map', rospy.Time(0))
		print trans
		print rot
	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		print "ERROR"
		continue

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
