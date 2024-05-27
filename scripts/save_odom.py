#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry

def callback(msg:Odometry):
    with open('./position.txt', 'a') as file:
        position = msg.pose.pose.position
        print(f'Position: x={position.x}, y={position.y}, z={position.z}')
        file.write(f'{position.x} {position.y} {position.z}\n')

if __name__ == '__main__':
    rospy.init_node('save_odom')
    rospy.Subscriber('/Odometry', Odometry, callback)
    rospy.spin()