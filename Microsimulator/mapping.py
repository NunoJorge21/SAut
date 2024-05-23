#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf.transformations
import numpy as np

def scan_callback(data: LaserScan) :
    # Convert the ranges list to a NumPy array
    #ranges_array = np.array(data.ranges)

    # Set the print options for the array
    #np.set_numeric_ops(threshold=np.inf, linewidth=120)

    # Print the array
    #rospy.loginfo("Received scan message with ranges array:")
    #rospy.loginfo(ranges_array)

    #return ranges_array

    return data.ranges

def pose_callback(data: PoseWithCovarianceStamped):

    quaternion = [data.pose.pose.orientation.x, data.pose.pose.orientation.y,
              data.pose.pose.orientation.z, data.pose.pose.orientation.w]
 
    euler = tf.transformations.euler_from_quaternion(quaternion)

    #rospy.loginfo("Received pose message with position: (%f, %f) and orientation yaw=%f",
    #              data.pose.pose.position.x, data.pose.pose.position.y, euler[2])
    
    pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler[2]]

    return pose

def listener():
    rospy.init_node('subscriber_node', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, scan_callback)
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, pose_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()