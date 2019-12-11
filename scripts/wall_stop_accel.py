#!/usr/bin/env python
import rospy,copy
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse
from pimouse_ros.msg import LightSensorValues

class WallStop():
    def __init__(self):
        self.cmd_vel = rospy.Publisher('/cmd_vel',Twist,queue_size=1)

        self.sensor_values = LightSensorValues()
        rospy.Subscriber('/lightsensors',LightSensorValues,self.callback)

    def callback(self,messages):
        self.sensor_values = messages

    def run(self):
        rate = rospy.Rate(10)
        data = Twist()

        accel = 0.01
        data.linear.x = 0.0
        while not rospy.is_shutdown():
            data.linear.x += accel

            if self.sensor_values.sum_all >= 700: data.linear.x = 0.0
            elif data.linear.x <= 0.1:           data.linear.x = 0.1
            elif data.linear.x >= 0.4:           data.linear.x = 0.4

            self.cmd_vel.publish(data)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('wall_stop')
    rospy.wait_for_service('/motor_on')
    rospy.wait_for_service('/motor_off')
    rospy.on_shutdown(rospy.ServiceProxy('/motor_off',Trigger).call)
    rospy.ServiceProxy('/motor_on',Trigger).call()
    WallStop().run()
