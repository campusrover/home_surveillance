#!/usr/bin/env python

from threading import Thread

import rospy
import actionlib
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class Zones:
  secondary_zones = ["B", "C", "D"]
  secondary_zones_priority_map = {"B": 0, "C": 1, "D": 2}
  lowest_priority = 2
  zone_map = {
    "A": [
      [ (-1.0, 0.0, 0.0),
        (0.0, 0.0, 0.0, 1.0)],
      [ (-1.0, 0.7, 0.0),
        (0.0, 0.0, 0.0, 1.0)]
    ],
    "B": [
      [ (1.0, 0.0, 0.0),
        (0.0, 0.0, 0.0, 1.0)],
      [ (1.0, 0.7, 0.0),
        (0.0, 0.0, 0.0, 1.0)]
    ],
    "C":  [
      [ (-1.0, 2.0, 0.0),
        (0.0, 0.0, 0.0, 1.0)],
      [ (-0.3, 2.0, 0.0),
        (0.0, 0.0, 0.0, 1.0)]
    ],
    "D": [
      [ (-1.0, -1.0, 0.0),
        (0.0, 0.0, 0.0, 1.0)],
      [ (-0.3, -1.0, 0.0),
        (0.0, 0.0, 0.0, 1.0)]
    ]}

class Patrol:
  def __init__(self, global_interrupt_map, local_interrupt_map, guard_name, zone):
    self.guard_name = guard_name
    self.zone = zone

    self.global_interrupt_map = global_interrupt_map

    self.local_interrupt_map = local_interrupt_map

    self.client = actionlib.SimpleActionClient(f'/{self.guard_name}/move_base', MoveBaseAction)

    self.client.wait_for_server()
  
  def execute(self):
    interrupt_handler_thread = Thread(target=self.__interrupt_handler, daemon = True)
    interrupt_handler_thread.start()
    while not self.__interrupt_triggered():
      for pose in self.zone:
        goal = self.__set_goal_pose(pose)
        print("Going for goal: ", goal)
        self.client.send_goal(goal)
        self.client.wait_for_result()
        if self.__interrupt_triggered():
          break
    interrupt_handler_thread.join()
  
  #=======Callback Methods=======

  def __interrupt_handler(self):
    while True:
      if self.__interrupt_triggered():
        self.client.cancel_goal()
        break

  #=======Helper Methods=======
  
  def __interrupt_triggered(self):
    for interrupt in self.global_interrupt_map.values():
      if interrupt.is_set():
        return True
    if self.local_interrupt_map[f"{self.guard_name}_shutdown"].is_set():
        return True
    return False

  def __set_goal_pose(self, pose):
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
  
  def __parse_message(self, msg):
    split_msg = msg.data.split(', ')
    return int(split_msg[0]), split_msg[1]