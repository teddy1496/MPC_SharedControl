#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, Twist, Pose
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from visualization_msgs.msg import Marker
import numpy as np
from tf.transformations import quaternion_from_euler, euler_from_quaternion


global X
global Y
global THETA
X = 0
Y = 0
THETA = 0

def angleWrapping(th):
    while th < -np.pi:
        th += 2 * np.pi
    while th > np.pi:
        th -= 2 * np.pi
    return th


def pose_clbk(data):
    global X
    global Y
    global THETA
    X = data.position.x
    Y = data.position.y
    THETA = euler_from_quaternion([data.orientation.x, data.orientation.y,
    data.orientation.z, data.orientation.w])[2]


def callback(data):
    global X
    global Y
    global THETA
    linear_vel_v = data.linear.x
    linear_vel_w = data.angular.z
    leader_gazebo = ModelState()

    dt = 0.01

    X = X + (linear_vel_v * np.cos(THETA) * dt)
    Y = Y + (linear_vel_v * np.sin(THETA) * dt)
    THETA = THETA + linear_vel_w * dt
    THETA = angleWrapping(THETA)

    quat = quaternion_from_euler(0,0,THETA)

    leader_gazebo.pose.position.x = X
    leader_gazebo.pose.position.y = Y
    leader_gazebo.pose.position.z = 0.05

    leader_gazebo.pose.orientation.x = quat[0]
    leader_gazebo.pose.orientation.y = quat[1]
    leader_gazebo.pose.orientation.z = quat[2]
    leader_gazebo.pose.orientation.w = quat[3]

    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state( leader_gazebo )

    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

    pub_square.publish(final_marker)

leader_marker = Marker()
final_marker = Marker()
final_marker.header.frame_id = "/map"
final_marker.ns = "square_final_area"
final_marker.id = 0
final_marker.type = leader_marker.CUBE
final_marker.action = 0
final_marker.pose.position.x = 4.0
final_marker.pose.position.y = 2.5
final_marker.pose.orientation.x = 0
final_marker.pose.orientation.y = 0
final_marker.pose.orientation.z = 0
final_marker.pose.orientation.w = 1
final_marker.scale.x = 2
final_marker.scale.y = 2
final_marker.scale.z = 0.01
final_marker.color.r = 0.0
final_marker.color.g = 1.0
final_marker.color.b = 1.0
final_marker.color.a = 0.5


# Intializes everything
def start():
    global pub_circle
    global pub_point
    global pub_square

    pub_square = rospy.Publisher('final_region', Marker, queue_size = 0)
    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber('/cmd_vel_leader', Twist, callback)
    rospy.Subscriber('/initial_pose', Pose, pose_clbk)

    # starts the node
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('Leader_move')
    start()
