#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

# Author: Andrew Dai
# This ROS Node converts Joystick inputs from the joy node
# into commands for turtlesim

# Receives joystick messages (subscribed to Joy topic)
# then converts the joysick inputs into Twist commands
# axis 1 aka left stick vertical controls linear speed
# axis 0 aka left stick horizonal controls angular speed



def callback(data):
    if_ending = Bool()
    if data.buttons[1] == 1:
    	if_ending = True
    else:
        if_ending = False

    pub.publish(if_ending)

# Intializes everything
def start():
    global pub
    pub = rospy.Publisher('/end_sim', Bool, queue_size = 15)
    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("joy", Joy, callback)
    # starts the node
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('Mir_Tele')
    start()
