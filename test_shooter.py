#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3
from m2_pr2020.srv import SetShoot

START = -3000
RELEASE_PT = -7000
STOP = -15000
ready = False
RANGE = 1000
def shoot_cb(data):
    global ready
    pos = abs(data.x)
    if ready:
        #print(pos)
        if pos >= (abs(RELEASE_PT) - RANGE ) and pos <= (abs(RELEASE_PT) + RANGE ):
            clip_io_pub.publish(True)
            print("Release")


rospy.init_node('test_shooter')

shooter_shoot_srv = rospy.ServiceProxy('shooter/trigger_shoot_pos', SetShoot)
clip_io_pub = rospy.Publisher('/io_board1/io_1/set_state', Bool, queue_size=1)
rospy.Subscriber("motor_shooter/p_feedback", Vector3, shoot_cb, queue_size=1)

rospy.sleep(1)

shooter_shoot_srv(START, 1000, 1000, 1000)

clip_io_pub.publish(True)
print("Ready to start")
inp = input("Clip (yes/no):")
clip_io_pub.publish(False)
inp = input("Shoot? (yes/no):")
if(inp=="yes"):
    rospy.sleep(1)
    ready = True

    shooter_shoot_srv(STOP, 4000, 9000, 11000)