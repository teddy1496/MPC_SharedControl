#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

# Author: Andrew Dai
# This ROS Node converts Joystick inputs from the joy node
# into commands for turtlesim

def callback(data):
    twist = Twist()

    if_moving = Bool()
    start_timing = False
    end_timing = False
    if (data.buttons[0] == 1):
        twist.linear.x = 0.8*data.axes[1]
    	twist.angular.z = 0.8*data.axes[0]
        if_moving = True

    elif (data.buttons[0] == 0):
        twist.linear.x = 0
        twist.angular.z = 0
        if_moving = False

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
    pub_moving.publish(if_moving)

# Intializes everything
def start():
    global pub_vel
    global pub_moving
    global pub_start_time
    global pub_end_time
    pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
    pub_moving = rospy.Publisher('/robot_moving', Bool, queue_size = 2)
    pub_start_time = rospy.Publisher('/timer_start', Bool, queue_size = 2)
    pub_end_time = rospy.Publisher('/timer_end', Bool, queue_size = 2)
    rospy.Subscriber("joy", Joy, callback)
    # starts the node
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('Mir_Tele')
    start()
