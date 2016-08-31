#!/usr/bin/env python

import sys, time
import rospy
from raspimouse_ros.srv import *
from raspimouse_ros.msg import *
from std_msgs.msg import UInt16

def switch_motors(onoff):
    rospy.wait_for_service('/raspimouse/switch_motors')
    try:
        p = rospy.ServiceProxy('/raspimouse/switch_motors', SwitchMotors)
        res = p(onoff)
        return res.accepted
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    else:
        return False

def raw_control(left_hz,right_hz):
    if not rospy.is_shutdown():
        d = LeftRightFreq()
        d.left = left_hz
        d.right = right_hz
        pub_motor.publish(d)


def lightsensor_callback(data):
    lightsensors.left_side = data.left_side
    lightsensors.right_side = data.right_side
    lightsensors.left_forward = data.left_forward
    lightsensors.right_forward = data.right_forward

def stop_motors():
    raw_control(0,0)
    switch_motors(False)

if __name__ == "__main__":
    rospy.init_node("lefthand")

    if not switch_motors(True):
        print "[check failed]: motors are not empowered"
        sys.exit(1)

    lightsensors = LightSensorValues()
    sub_ls = rospy.Subscriber('/raspimouse/lightsensors', LightSensorValues, lightsensor_callback)
    pub_motor = rospy.Publisher('/raspimouse/motor_raw', LeftRightFreq, queue_size=10)

    rospy.on_shutdown(stop_motors)

    r = rospy.Rate(10)
    wall = False
    while not rospy.is_shutdown():
        # adjustment of front range
        base = 500
        target = 2500
        front = lightsensors.left_forward + lightsensors.right_forward
        diff = target - front
        e = 0.2

        # adjustment of direction
        sdiff = lightsensors.right_forward - lightsensors.left_forward
        se = 0.2

        raw_control(e*diff + se*sdiff,e*diff - se*sdiff)

        r.sleep()

    stop_motors()
