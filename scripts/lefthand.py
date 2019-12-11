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

def left_walltrace(ls):
    if too_right(ls):
        turn_left()
        return
    if too_left(ls):
        turn_right()
        return

    base = 500
    e = 0.2 * (ls.left_side - 700)
    raw_control(base + e,base - e)

def turn_right():
    raw_control(250,-250)

def turn_left():
    raw_control(-250,250)

def turn_to_open_space(ls,r):
    right = False
    if ls.left_side > ls.right_side:
        right = True

    while ls.left_forward > 500 or ls.right_forward > 500:
        if right: turn_right()
        else: turn_left()
        r.sleep()

    raw_control(0,0)

def stop_motors():
    raw_control(0,0)
    switch_motors(False)

def dead_end(ls):
    th = 700
    return ls.right_side > th \
        and ls.left_side > th \
        and ls.right_forward > th \
        and ls.left_forward > th 

def too_right(ls):
    return ls.right_side > 2000

def too_left(ls):
    return ls.left_side > 2000

def still_wall(ls):
    return ls.left_forward > 500 or ls.right_forward  > 500

def find_wall(ls):
    return ls.left_forward > 1500 or ls.right_forward > 1500

if __name__ == "__main__":
    rospy.init_node("lefthand")
    rospy.on_shutdown(stop_motors)

    if not switch_motors(True):
        print "[check failed]: motors are not empowered"
        sys.exit(1)

    lightsensors = LightSensorValues()
    pub_motor = rospy.Publisher('/raspimouse/motor_raw', LeftRightFreq, queue_size=10)
    subls = rospy.Subscriber('/raspimouse/lightsensors', LightSensorValues, lightsensor_callback)

    r = rospy.Rate(10)

    wall = False
    while not rospy.is_shutdown():
        if dead_end(lightsensors):
            turn_to_open_space(lightsensors,r)
        elif wall:
            turn_left()
            wall = still_wall(lightsensors)
        else:
            left_walltrace(lightsensors)
            wall = find_wall(lightsensors)
            
        r.sleep()

    stop_motors()
