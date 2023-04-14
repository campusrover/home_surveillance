#!/usr/bin/env python

import time
import rospy
from std_msgs.msg import String

i = 0

def hegemony_handler(msg):
    global i
    i += 1
    print(f'{msg.data}, {i}')

if __name__== '__main__':
    rospy.init_node("lea_tester")
    mps = {'roba', 'robb', 'robc', 'rafael'}
    time.sleep(5)
    for mp in mps:
        rospy.Subscriber(f'{mp}_hegemony_broadcasts', String, hegemony_handler)
    rospy.spin()
    