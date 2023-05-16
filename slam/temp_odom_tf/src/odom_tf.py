#!/usr/bin/env python3
import tf
import rospy
import numpy as np
from geometry_msgs.msg import Pose
import tf_helper
from nav_msgs.msg import Odometry

last_pose = np.array([0,0,0])

def update(odom:Odometry):
    global last_pose
    currentState = odom.pose.pose
    last_pose[0] = currentState.position.x
    last_pose[1] = currentState.position.y
    quaternion = [currentState.orientation.x, currentState.orientation.y, currentState.orientation.z, currentState.orientation.w ]
    last_pose[2] = tf.transformations.euler_from_quaternion(quaternion)[2]  


rospy.init_node("transformer")
tfHelper = tf_helper.TFHelper("transformer")

rospy.Subscriber("/aft_mapped_to_init", Odometry, callback=update)

# Broadcast tf
while not rospy.is_shutdown():
    br = tf.TransformBroadcaster()
    yaw = -last_pose[2]
    s, c = np.sin(yaw), np.cos(yaw)
    rot_mat = np.array([[c,s],[-s,c]])
    pose = rot_mat@(last_pose[:2].reshape(2,1))

    br.sendTransform((-pose[0][0], -pose[1][0], 0),
    tf.transformations.quaternion_from_euler(0, 0, -yaw),
    rospy.Time.now(),
    "map","velodyne")