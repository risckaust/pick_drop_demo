#!/usr/bin/env python

# ROS python API
import rospy
import tf
import time

def main():
	# initiate node
        rospy.init_node('tf_listner_test', anonymous=True)
        listener = tf.TransformListener()
        time.sleep(1)
        #(trans,rot) = listener.lookupTransform('/disc', '/map', rospy.Time(0))
	try:
                t1 = time.time()
		(trans,rot) = listener.lookupTransform('/disc', '/map', rospy.Time(0))
                t = time.time()-t1
                print "duration(s):" , t
		print "Translation: ", trans
		print "Rotation:", rot
	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		print "ERROR"
		

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
