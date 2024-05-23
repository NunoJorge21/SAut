#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf.transformations
import numpy as np

def scan_callback(data: LaserScan) :
    return data.ranges

def pose_callback(data: PoseWithCovarianceStamped):
    quaternion = [data.pose.pose.orientation.x, data.pose.pose.orientation.y,
                  data.pose.pose.orientation.z, data.pose.pose.orientation.w]
    euler = tf.transformations.euler_from_quaternion(quaternion)
    pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler[2]]
    return pose

def main():
    rospy.init_node('subscriber_node', anonymous=True)
    while not rospy.is_shutdown():
        scan_data = rospy.wait_for_message('/scan', LaserScan)
        pose_data = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped)
        ranges = scan_callback(scan_data)
        pose = pose_callback(pose_data)

if __name__ == '__main__':
    main()
