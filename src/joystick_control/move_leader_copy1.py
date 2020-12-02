#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, Twist, Pose
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from visualization_msgs.msg import Marker
import numpy as np
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion


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
    THETA = euler_from_quaternion([data.orientation.x,
                                    data.orientation.y,
                                    data.orientation.z,
                                    data.orientation.w])[2]


def callback(data):
    global X
    global Y
    global THETA
    linear_vel_v = data.linear.x
    linear_vel_w = data.angular.z
    leader_gazebo = ModelState()
    leader_gazebo.model_name = 'leader'
    leader_pose.header.stamp = rospy.Time.now()
    leader_marker.header.stamp = rospy.Time.now()

    dt = 0.01

    X = X + (linear_vel_v * np.cos(THETA) * dt)
    Y = Y + (linear_vel_v * np.sin(THETA) * dt)
    THETA = THETA + linear_vel_w * dt
    THETA = angleWrapping(THETA)

    quat = quaternion_from_euler(0,0,THETA)

    leader_pose.pose.position.x = X
    leader_pose.pose.position.y = Y
    leader_pose.pose.orientation.x = quat[0]
    leader_pose.pose.orientation.y = quat[1]
    leader_pose.pose.orientation.z = quat[2]
    leader_pose.pose.orientation.w = quat[3]

    leader_gazebo.pose.position.x = X
    leader_gazebo.pose.position.y = Y
    leader_gazebo.pose.orientation.x = quat[0]
    leader_gazebo.pose.orientation.y = quat[1]
    leader_gazebo.pose.orientation.z = quat[2]
    leader_gazebo.pose.orientation.w = quat[3]

    leader_marker.pose.position.x = X
    leader_marker.pose.position.y = Y

    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state',
					SetModelState)
        resp = set_state( leader_gazebo )

    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

    pub_circle.publish(leader_marker)
    # pub_square.publish(final_marker)
    pub_point.publish(leader_pose)


leader_pose = PoseStamped()
leader_pose.header.frame_id = "/map"

leader_marker = Marker()
leader_marker.header.frame_id = "/map"
leader_marker.ns = "cicle_for_leader"
leader_marker.id = 0
leader_marker.type = leader_marker.CYLINDER
leader_marker.action = 0
leader_marker.pose.orientation.x = 0
leader_marker.pose.orientation.y = 0
leader_marker.pose.orientation.z = 0
leader_marker.pose.orientation.w = 1
leader_marker.scale.x = 0.88
leader_marker.scale.y = 0.88
leader_marker.scale.z = 0.01
leader_marker.color.r = 0.0
leader_marker.color.g = 1.0
leader_marker.color.b = 0.0
leader_marker.color.a = 1.0

# final_marker = Marker()
# final_marker.header.frame_id = "/map"
# final_marker.ns = "square_final_area"
# final_marker.id = 0
# final_marker.type = leader_marker.CUBE
# final_marker.action = 0
# final_marker.pose.position.x = 4.0
# final_marker.pose.position.y = 2.5
# final_marker.pose.orientation.x = 0
# final_marker.pose.orientation.y = 0
# final_marker.pose.orientation.z = 0
# final_marker.pose.orientation.w = 1
# final_marker.scale.x = 2
# final_marker.scale.y = 2
# final_marker.scale.z = 0.01
# final_marker.color.r = 0.0
# final_marker.color.g = 1.0
# final_marker.color.b = 1.0
# final_marker.color.a = 0.5


# Intializes everything
def start():
    global pub_circle
    global pub_point
    # global pub_square

    pub_point = rospy.Publisher('/leader_point', PoseStamped,
				queue_size = 3)
    pub_circle = rospy.Publisher('circle_leader', Marker,
				queue_size = 0)
    # pub_square = rospy.Publisher('final_region', Marker,
	# 			queue_size = 0)
    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber('/cmd_vel_leader', Twist, callback)
    rospy.Subscriber('/initial_pose', Pose, pose_clbk)

    # starts the node
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('Leader_move')
    start()
