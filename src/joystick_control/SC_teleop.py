#!/usr/bin/env python
# This code is an extract from the Jpystick Control
# node from Andrew Dai

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool


def callback(data):
    twist = Twist()
    if_ending = Bool()

    start_timing = False
    end_timing = False
    if (data.buttons[0] == 1 and data.buttons[1] == 0):
        if_ending = False
        twist.linear.x = 0.8*data.axes[1]
    	twist.angular.z = 0.8*data.axes[0]

    elif (data.buttons[0] == 0 and data.buttons[1] == 0):
        if_ending = False
        twist.linear.x = 0
        twist.angular.z = 0

    elif (data.buttons[1] == 1):
        if_ending = True
        twist.linear.x = 0
        twist.angular.z = 0

    if(data.buttons[3] == 1):
        start_timing = True
    elif (data.buttons[4] == 1):
        end_timing = True
    else:
        start_timing = False
        end_timing = False

    pub_start_time.publish(start_timing)
    pub_end_time.publish(end_timing)
    pub_vel.publish(twist)
    pub_termin.publish(if_ending)

# Intializes everything
def start():
    global pub_vel
    global pub_termin
    global pub_start_time
    global pub_end_time
    pub_vel = rospy.Publisher('/cmd_vel_leader', Twist,
				queue_size = 10)
    pub_termin = rospy.Publisher('/end_sim', Bool,
				queue_size = 2)
    pub_start_time = rospy.Publisher('/timer_start', Bool,
                queue_size = 2)
    pub_end_time = rospy.Publisher('/timer_end', Bool,
                queue_size = 2)
    rospy.Subscriber("joy", Joy, callback)
    # starts the node
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('Mir_Tele')
    start()
