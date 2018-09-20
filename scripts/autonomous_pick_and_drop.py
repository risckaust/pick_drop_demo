#!/usr/bin/env python

# ROS python API
import rospy

# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped

# apriltag2 messages
from apriltags2_ros.msg import AprilTagDetection, AprilTagDetectionArray

# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from std_msgs.msg import Bool

from sensor_msgs.msg import Joy


import tf

import time

# Flight modes class
# Flight modes are activated using ROS services
class fcuModes:
    def __init__(self):
        pass

    def setArm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException, e:
            print "Service arming call failed: %s"%e

    def setDisarm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(False)
        except rospy.ServiceException, e:
            print "Service disarming call failed: %s"%e

    def setStabilizedMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='STABILIZED')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Stabilized Mode could not be set."%e

    def setOffboardMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Offboard Mode could not be set."%e

    def setAltitudeMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='ALTCTL')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Altitude Mode could not be set."%e

    def setPositionMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='POSCTL')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Position Mode could not be set."%e

    def setAutoLandMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.LAND')
        except rospy.ServiceException, e:
              print "service set_mode call failed: %s. Autoland Mode could not be set."%e

# Main class: State machine
# States; ['Takeoff', 'go to search point', 'pick', 'go to drop point', 'drop']
class Controller:
    # initialization method
    def __init__(self):
        # Drone state
        self.state = State()
        # Instantiate a setpoints message
        self.sp         = PositionTarget()
        # set the flag to use position setpoints and yaw angle
        self.sp.type_mask    = int('010111111000', 2)
        # LOCAL_NED
        self.sp.coordinate_frame= 1

        # We will fly at a fixed altitude for now
        # Altitude setpoint, [meters]
        self.ALT_SP        = 1
        # update the setpoint message with the required altitude
        self.sp.position.z    = self.ALT_SP

        # Fence. We will assume a square fence for now
        self.FENCE_LIMIT_X = 2.0
        self.FENCE_LIMIT_Y = 3.0
        self.FENCE_LIMIT_Z_DOWN = 0.25
        self.FENCE_LIMIT_Z_UP = self.ALT_SP

        # A Message for the current local position of the drone
        self.local_pos = Point(0.0, 0.0, 0.0)

        self.modes = fcuModes()

        # States
        self.TAKEOFF	= 0
        self.SEARCH		= 0
        self.PICK		= 0
        self.DROPPOINT	= 0
        self.DROP		= 0
        self.PRE_LAND	= 0
        self.LAND		= 0

        # Pick/drop altitudes
        self.PICK_ALT	= 0.25 # meter(s)
        self.DROP_ALT	= 0.25 # meter(s)

        # X/Y coordinates of search point where object is expected to be around
        self.SEARCH_POINT_X = 0.5
        self.SEARCH_POINT_Y = -2.17

        # object position in local map
        # can be found from tf tree
        self.OBJ_POS_X = 0.0
        self.OBJ_POS_Y = 0.0
        self.OBJ_POS_Z = 0.0

        # Object picking state
        self.IS_OBJECT_PICKED = False

        self.IS_OBJECT_DETECTED = False

        # Drop point coordinates
        self.DROP_POINT_X = -1.89
        self.DROP_POINT_Y = -2.24
        self.DROP_POINT_Z = self.ALT_SP

        # Land point
        self.LAND_POINT_X = -0.46
        self.LAND_POINT_Y = -2.25

    def resetStates(self):
    	self.TAKEOFF	= 0
        self.SEARCH		= 0
        self.PICK		= 0
        self.DROPPOINT	= 0
        self.DROP		= 0
        self.PRE_LAND	= 0
        self.LAND		= 0

    def bound(self, v, low, up):
			r = v
			if v > up:
				r = up
			if v < low:
				r = low

			return r

    # Callbacks

    ## local position callback
    def posCb(self, msg):
        self.local_pos.x = msg.pose.position.x
        self.local_pos.y = msg.pose.position.y
        self.local_pos.z = msg.pose.position.z

    ## Drone State callback
    def stateCb(self, msg):
        self.state = msg

    # object position callback
    # expecting only single object
    def gripperCb(self, msg):
    	if msg is not None:
    		self.IS_OBJECT_PICKED = msg.data



    ## Update setpoint message
    def updateSp(self):
    	if not (len(self.joy_msg.axes)>0 and len(self.joy_msg.buttons )>0 ):
    		return

        x = 1.0*self.joy_msg.axes[3]
        y = 1.0*self.joy_msg.axes[2]
        z = self.joy_msg.axes[1]
      
        #self.sp.position.x = self.local_pos.x + self.STEP_SIZE*x
        #self.sp.position.y = self.local_pos.y + self.STEP_SIZE*y

        xsp = self.local_pos.x + self.STEP_SIZE*x
        ysp = self.local_pos.y + self.STEP_SIZE*y
        zsp = self.local_pos.z + self.STEP_SIZE*z

        # limit
        self.sp.position.x = self.bound(xsp, -1.0*self.FENCE_LIMIT_X, self.FENCE_LIMIT_X)
        self.sp.position.y = self.bound(ysp, -1.0*self.FENCE_LIMIT_Y, self.FENCE_LIMIT_Y)
        self.sp.position.z = self.bound(zsp, self.FENCE_LIMIT_Z_DOWN, self.FENCE_LIMIT_Z_UP)

# Main function
def main():

    # initiate node
    rospy.init_node('setpoint_node', anonymous=True)

    # controller object
    cnt = Controller()

    # ROS loop rate, [Hz]
    rate = rospy.Rate(20.0)

    # Subscribe to drone state
    rospy.Subscriber('mavros/state', State, cnt.stateCb)

    # Subscribe to drone's local position
    rospy.Subscriber('mavros/local_position/pose', PoseStamped, cnt.posCb)

    # Gripper state subscriber
    rospy.Subscriber("/gripper_status", Bool, cnt.gripperCb)

    # Setpoint publisher
    sp_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)

    joy_pub = rospy.Publisher('/joy', Joy, queue_size=1)

    # We need to send few setpoint messages, then activate OFFBOARD mode, to take effect
    k=0
    while k<10:
        sp_pub.publish(cnt.sp)
        rate.sleep()
        k = k+1

    # Prepare for a hover at current point
    cnt.sp.position.x = cnt.local_pos.x
    cnt.sp.position.y = cnt.local_pos.y
    cnt.sp.position.z = cnt.ALT_SP

    # reset state machine
    cnt.resetStates()

    # start with TAKEOFF state
    cnt.TAKEOFF = 1

    # activate OFFBOARD mode
    #modes.setOffboardMode()

    # tf listner: To find the transfomation (translatin, rotation) between map -> object
    listener = tf.TransformListener()

    PICK_TRIAL = 0
    MAX_PICK_TRIALS = 3

    # flag to re-pick if failed
    RE_PICK = 0

    # ROS main loop
    while not rospy.is_shutdown():
        if cnt.TAKEOFF:
            rospy.loginfo("TAKEOFF state")
            if abs(cnt.local_pos.z - cnt.ALT_SP) < 0.1:
                cnt.resetStates()
                cnt.SEARCH = 1
                search_t = time.time()

        if cnt.SEARCH:
            rospy.loginfo("SEARCH state")
            cnt.IS_OBJECT_DETECTED = False
            cnt.sp.position.x = cnt.SEARCH_POINT_X
            cnt.sp.position.y = cnt.SEARCH_POINT_Y

            # TODO: find object in tf tree
            try:
            	x,y = 0., 0.
            	for i in range(100):
					(trans,rot) = listener.lookupTransform('/map', '/disc', rospy.Time(0))
					x = x + trans[0]
					y = y + trans[1]
				trans[0] = x/100.
				trans[1] = y/100.
                cnt.IS_OBJECT_DETECTED = True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                cnt.IS_OBJECT_DETECTED = False

            # check distance to point
            err = (cnt.sp.position.x - cnt.SEARCH_POINT_X)**2 + (cnt.sp.position.y - cnt.SEARCH_POINT_Y)**2
            # Take square root
            err = err**0.5
            if err < 0.05 and cnt.IS_OBJECT_DETECTED:
                cnt.resetStates()
                cnt.PICK = 1
                # start picking duration
                pick_t = time.time()

            # if search time exceeded with no detection, go to land state
            if (time.time() - search_t) > 30.0:
                cnt.resetStates()
                cnt.PRE_LAND = 1




        if cnt.PICK:
            rospy.loginfo("PICK state")

            cnt.sp.position.x = trans[0] - 0.03
            cnt.sp.position.y = trans[1] - 0.03
            cnt.sp.position.z = trans[2]

            # in case of re-pick
            if RE_PICK:
            	rospy.loginfo("RE-PICK state")
                cnt.sp.position.z = cnt.ALT_SP
                if abs(cnt.local_pos.z - cnt.ALT_SP) < 0.1:
                    RE_PICK = 0

            if cnt.IS_OBJECT_PICKED:
                cnt.resetStates()
                cnt.DROPPOINT = 1

            if (time.time() - pick_t) > 5.0 and not cnt.IS_OBJECT_PICKED and not RE_PICK:
                RE_PICK = 1
                pick_t = time.time()
                PICK_TRIAL  = PICK_TRIAL + 1

            # In case picking has failed, go to land state
            if PICK_TRIAL > MAX_PICK_TRIALS:
                cnt.resetStates()
                cnt.PRE_LAND = 1

        if cnt.DROPPOINT:
            rospy.loginfo("DROPPOINT state")
            cnt.sp.position.x = cnt.DROP_POINT_X
            cnt.sp.position.y = cnt.DROP_POINT_Y
            cnt.sp.position.z = cnt.DROP_POINT_Z

            err = (cnt.local_pos.x - cnt.DROP_POINT_X)**2 + (cnt.local_pos.y - cnt.DROP_POINT_Y)**2
            err = err**0.5
            if err < 0.05:
                cnt.resetStates()
                cnt.DROP = 1

        if cnt.DROP:
            rospy.loginfo("DROP state")
            # TODO: publish apropriate Joy msg to send drop signal
            joy_msg = Joy()
            joy_msg.header.stamp = rospy.Time.now()
            joy_msg.axes = [0.0, 0.0, 0.0, 0.0, 0.0, -1.0]
            joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
            joy_pub.publish(joy_msg)

            if not cnt.IS_OBJECT_PICKED:
                cnt.resetStates()
                cnt.PRE_LAND = 1

        if cnt.PRE_LAND:
            rospy.loginfo("PRE-LAND state")
            cnt.sp.position.x = cnt.LAND_POINT_X
            cnt.sp.position.y = cnt.LAND_POINT_Y
            cnt.sp.position.z = cnt.ALT_SP

            err = (cnt.local_pos.x - cnt.LAND_POINT_X)**2 + (cnt.local_pos.y - cnt.LAND_POINT_Y)**2
            err = err**0.5

            if err < 0.1:
                cnt.resetStates()
                cnt.LAND = 1

        if cnt.LAND:
            rospy.loginfo("LAND state")
            cnt.modes.setAutoLandMode()
            break


        sp_pub.publish(cnt.sp)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
