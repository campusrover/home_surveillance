#!/usr/bin/env python
import rospy
import actionlib
from std_msgs.msg import String

# move_base is the package that takes goals for navigation
# there are different implemenetations with a common interface
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# You need to know the coordinates that the map you are working
# in is. I experimented with these numbers and the turtlebot3_stage_4
# map from Turtlebot3. The first array is x,y,z location. The second one
# is a "quaternion" defining an orientation. Quaternions are a different
# mathematical represetnation for "euler angles", yaw, pitch and roll.

class PatrolTester:
  def __init__(self):
    self.waypoints = [
        [ (-1.0, 0.0, 0.0),
          (0.0, 0.0, 0.0, 1.0)],
        [ (-1.0, 2.0, 0.0),
          (0.0, 0.0, 0.0, 1.0)]
    ]
    self.should_stop_patrol = False
    # A node called 'patrol' which is an action client to move_base
    rospy.init_node('leader_patrol')
    rospy.Subscriber('keys', String, self.key_input_handler)
    self.client = actionlib.SimpleActionClient('/roba/move_base', MoveBaseAction)

    # wait for action server to be ready
    self.client.wait_for_server()
  
  def key_input_handler(self, msg):
    if msg.data[0] == 'c':
      self.client.cancel_goal()
      self.should_stop_patrol = True

# Function to generate a proper MoveBaseGoal() from a two dimensional array
# containing a location and a rotation. This is just to make the waypoints array
# simpler
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
      for pose in self.waypoints:
        goal = self.goal_pose(pose)
        print("Going for goal: ", goal)
        self.client.send_goal(goal)
        self.client.wait_for_result()
        if self.should_stop_patrol:
          break

# Main program starts here
if __name__ == '__main__':
  pt = PatrolTester()
  pt.run()