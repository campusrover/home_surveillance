#!/usr/bin/env python

import time
import rospy
from std_msgs.msg import String

i = 0

def election_result_handler(msg):
    global i
    i += 1
    print(f'{msg.data}, {i}')

if __name__== '__main__':
    rospy.init_node("lea_tester")
    mps = {'roba', 'robb', 'robc', 'rafael'}
    for mp in mps:
        rospy.Subscriber(f'{mp}_election_result_broadcasts', String, election_result_handler)
    rospy.spin()
    