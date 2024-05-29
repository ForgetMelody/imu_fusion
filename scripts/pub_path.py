#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

path_points = []

if __name__ == '__main__':
    rospy.init_node('pub_path', anonymous=True)
    pub = rospy.Publisher('/fast_lio_path', Path, queue_size=10)
    with open('/home/sosilent/catkin_ws/data/paths/fast_lio.txt', 'r') as f:
        path_points = np.loadtxt(f)
    path = Path()
    path.header.frame_id = "odom"
    for point in path_points:
        pose = PoseStamped()

        pose.header.frame_id = "odom"
        pose.pose.position.x = point[0]
        pose.pose.position.y = point[1]
        pose.pose.position.z = 0.0

        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0

        path.poses.append(pose)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub.publish(path)
        rate.sleep()
    