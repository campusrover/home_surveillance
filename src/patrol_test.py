#!/usr/bin/env python

from threading import Thread

import rospy
import actionlib
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class Zones:
  A = [
    [ (-1.0, 0.0, 0.0),
      (0.0, 0.0, 0.0, 1.0)],
    [ (-1.0, 0.7, 0.0),
      (0.0, 0.0, 0.0, 1.0)]
  ]
  B = [
    [ (1.0, 0.0, 0.0),
      (0.0, 0.0, 0.0, 1.0)],
    [ (1.0, 0.7, 0.0),
      (0.0, 0.0, 0.0, 1.0)]
  ]
  C = [
    [ (-1.0, 2.0, 0.0),
      (0.0, 0.0, 0.0, 1.0)],
    [ (-0.3, 2.0, 0.0),
      (0.0, 0.0, 0.0, 1.0)]
  ]
  D = [
    [ (-1.0, -1.0, 0.0),
      (0.0, 0.0, 0.0, 1.0)],
    [ (-0.3, -1.0, 0.0),
      (0.0, 0.0, 0.0, 1.0)]
  ]

class Patrol:
  def __init__(self, guard_name, zone):
    self.zone = zone

    self.should_stop_patrol = False

    rospy.Subscriber('keys', String, self.key_input_handler)
    self.client = actionlib.SimpleActionClient(f'/{guard_name}/move_base', MoveBaseAction)

    self.client.wait_for_server()
  
  def key_input_handler(self, msg):
    if msg.data[0] == 'c':
      self.client.cancel_goal()
      self.should_stop_patrol = True

  def goal_pose(self, pose):
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
  
  def run(self):
    while not self.should_stop_patrol:
      for pose in self.zone:
        goal = self.goal_pose(pose)
        print("Going for goal: ", goal)
        self.client.send_goal(goal)
        self.client.wait_for_result()
        if self.should_stop_patrol:
          break

# Main program starts here
if __name__ == '__main__':
  rospy.init_node('patrol_test')
  patrols = {Patrol('roba', Zones.A), Patrol('robb', Zones.B),
            Patrol('robc', Zones.C), Patrol('rafael', Zones.D) }

  patrol_threads = set()
  for patrol in patrols:
    patrol_threads.add(Thread(target=patrol.run, daemon = True))

  for thread in patrol_threads:
    thread.start()
  
  for thread in patrol_threads:
    thread.join()