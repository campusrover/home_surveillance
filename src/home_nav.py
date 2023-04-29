#!/usr/bin/env python

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

#=====Important: Maybe have a HomeNav class that wraps both the leader 
#=====management system and the HomeNavRoutine?====

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
        self.__set_guards(leader)
        self.__set_room_coordinates()
        self.__connect_to_move_base_clients()

    def run(self):
        self.__patrol(leader, rooms[0])
        i = 1
        for follower in self.followers:
            self.__patrol(follower, rooms[i])
            i+=1
        

    #=======Run Helper Methonds=======
    def __patrol(self, guard, room):
        for pose in room:
            goal = self.__make_goal_pose(pose)
            client = self.move_base_client[guard]
            client.send_goal(goal)
            client.wait_for_result()

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

    #=======Init Helper Methods=======
    def __set_guards(self, leader):
        self.leader = leader
        self.followers = {'roba', 'robb', 'robc', 'rafael'}
        self.followers.remove(self.leader)
    
    def __set_room_coordinates(self):
        self.rooms = []
        self.rooms.append([[(-1.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)],
                       [(-1.0, 2.0, 0.0), (0.0, 0.0, 0.0, 1.0)]])

    def __connect_to_move_base_clients(self):
        self.move_base_client = {}
        self.move_base_client[self.leader] = actionlib.SimpleActionClient(f'/{self.cur_leader}/move_base', MoveBaseAction)
        self.move_base_client[self.leader].wait_for_server()

        for follower in followers:
            self.move_base_client[follower] = actionlib.SimpleActionClient(f'/{follower}/move_base', MoveBaseAction)
            self.move_base_client[self.leader].wait_for_server()

if __name__ == '__main__':
    rospy.init_node('homeNav')

    mp = MP(robot_name)
    mp.attend_session()