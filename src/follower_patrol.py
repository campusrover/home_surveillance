#!/usr/bin/env python
import rospy
import actionlib

# move_base is the package that takes goals for navigation
# there are different implemenetations with a common interface
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# Function to generate a proper MoveBaseGoal() from a two dimensional array
# containing a location and a rotation. This is just to make the waypoints array
# simpler.

def goal_pose(pose):
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'robot1/base_link'
    return goal_pose


# Main program starts here
if __name__ == '__main__':

    # A node called 'patrol' which is an action client to move_base
    rospy.init_node('patrol')
    client = actionlib.SimpleActionClient('/robot2/move_base', MoveBaseAction)

    # wait for action server to be ready
    client.wait_for_server()

    # Loop until ^c
    while not rospy.is_shutdown():
        goal = goal_pose(pose)
        print("Going for goal: ", goal)
        client.send_goal(goal)
        client.wait_for_result()