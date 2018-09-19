# pick_drop_demo
This package can be used to demonstrate object transportation using a drone in an indoor setup. The control can be either via a human pilot who controls the drone using a joystick to fly the drone, pick, transport and drop the object, or fully autonomous using vision feedback.

The following setup is assumed.

* Indoor localization system (optitrack)
* A drone that is equipped with a PX4 autopilot and an arduino-controlled customized gripper.
* An object that is magnetic (can be picked by a permanent magnet)
* A ROS-compatible joystick for manual control
* A ROS-compatible camera for vision feedback, for autonomous mission
* ROS Kinetic, Ubuntu 16 on ODROID XU4, or a similar onboard computer

# Dependencies
* [vrpn_client_ros](http://wiki.ros.org/vrpn_client_ros)
* [apriltag2_ros](https://github.com/dmalyuta/apriltags2_ros)
* [cv_bridge](http://wiki.ros.org/cv_bridge)
* [rosserial](http://wiki.ros.org/rosserial)

# Installation
* Make sure you install the required dependencies above
* clone this package into your `~/catkin_ws/src` and build it
* The arduino code that controls the gripper is at  https://github.com/risckaust/pick_drop_demo/tree/master/gripper_joystick

# Experiments
* Place markers rigidly on the drone, and define a rigid body in *Motive*
* stream the rigid body info using VRPN, and make sure that **Up** axis is the **z-axis**
* It is assumed that you have an onboard computer which runs [mavros](http://wiki.ros.org/mavros), which can be used to feed the rigidbody pose from motion capture information to PX4.
* Example of how to get pose of a rigidbody from mocap to ROS: https://github.com/risckaust/pick_drop_demo/blob/master/launch/start_system.launch#L25-L27
* Example on how to relay it to mavros plugin: https://github.com/risckaust/pick_drop_demo/blob/master/launch/start_system.launch#L29
* **NOTE** Always double check that you can hover the drone in **POSITION** flight mode, before you execute the experiments in **OFFBOARD** mode.

## Manula control
* Adjust the rigidbody name in https://github.com/risckaust/pick_drop_demo/blob/master/launch/start_system.launch#L12
* Adjust connection links in https://github.com/risckaust/pick_drop_demo/blob/master/launch/start_system.launch#L3-L10
* Make sure that you give the joystick permissions (we used Logitech F710). The right stick is for x/y motion. The left stick is for height. The  red button is for disarm. The green button is for autoland. The down arrow is for dropping, if the object is picked (detected by the button on the gripper).
```
sudo chmod a+rw /dev/input/js0
```
run the following on the onboard computer
```bash
roslaunch pick_drop_demo start_system.launch
```

## Autonomous Mission
To be completed
