import rospy
import time
from std_msgs.msg import Bool

global start_time
global end_time


def start_timer_clbk(data):
    global start_time
    if data.data == True:
        start_time = time.time()
    else:
        pass

def end_timer_clbk(data):
    global start_time
    global end_time
    if data.data == True:
        end_time = time.time()
        print("Total time is: ", end_time-start_time)
        rospy.signal_shutdown("Task Completed.")
    else:
        pass


def start():
    rospy.Subscriber('/timer_start', Bool, start_timer_clbk)
    rospy.Subscriber('/timer_end', Bool, end_timer_clbk)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('time_calculator')
    start()
