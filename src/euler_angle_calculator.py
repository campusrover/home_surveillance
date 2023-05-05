#!/usr/bin/env python

import rospy
import tf

from geometry_msgs.msg import PoseWithCovarianceStamped

def quaternion_handler(msg):
    pose_with_covariance = msg.pose
    # Extract the quaternion from the PoseWithCovariance message
    quaternion_x = pose_with_covariance.pose.orientation.x
    quaternion_y = pose_with_covariance.pose.orientation.y
    quaternion_z = pose_with_covariance.pose.orientation.z
    quaternion_w = pose_with_covariance.pose.orientation.w
    
    # Convert the quaternion to Euler angles (roll, pitch, yaw)
    euler_angles = tf.transformations.euler_from_quaternion([quaternion_x, quaternion_y, quaternion_z, quaternion_w])

    # Extract the yaw angle (rotation around the z-axis)
    yaw_angle = euler_angles[2]
    print(f"Yaw angle is: {yaw_angle}")

rospy.init_node('euler_angle_calculator', anonymous=True)
sub = rospy.Subscriber('/rafael/initialpose', PoseWithCovarianceStamped, quaternion_handler)

rospy.spin()