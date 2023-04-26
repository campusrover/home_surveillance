#!/usr/bin/env python

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


#=====Leader management system=====
# implement as a class?
leader_changed = False
cur_leader = ''

for guard in GUARDS:
            rospy.Subscriber(f'{guard}_election_result_broadcasts', String, election_result_handler)

    def election_result_handler(self, msg):
        new_leader = msg.data
        if cur_leader == '':
            cur_leader = new_leader
        else if self.cur_leader != new_leader:
            leader_changed = True
            cur_leader = new_leader
#=================================

class HomeNavRoutine:
    def __init__(self, leader):
        self.cur_leader = leader
        self.cur_followers = {'roba', 'robb', 'robc', 'rafael'}
        self.cur_followers.remove(self.cur_leader)
        self.room_1 = [[(-1.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)],
                       [(-1.0, 2.0, 0.0), (0.0, 0.0, 0.0, 1.0)]]
        self.room_2 = []
        self.room_3 = []
        self.room_4 = []

    def run(self):
        # initialize action clients
        # execute move_base goals
        # exceptions?

    #=======Helper Methods=======

    def __make_goal_pose(self, pose):
        goal_pose = MoveBaseGoal()
        goal_pose.target_pose.header.frame_id = 'map'
        goal_pose.target_pose.pose.position.x = pose[0][0]
        goal_pose.target_pose.pose.position.y = pose[0][1]
        goal_pose.target_pose.pose.position.z = pose[0][2]
        goal_pose.target_pose.pose.orientation.x = pose[1][0]
        goal_pose.target_pose.pose.orientation.y = pose[1][1]
        goal_pose.target_pose.pose.orientation.z = pose[1][2]
        goal_pose.target_pose.pose.orientation.w = pose[1][3]
        return goal_pose

if __name__ == '__main__':
    rospy.init_node('homeNav')

    mp = MP(robot_name)
    mp.attend_session()