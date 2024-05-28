#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry

def callback(msg:Odometry):
    #delete the previous file
    
    with open('./imu_odom.txt', 'a') as file:
        position = msg.pose.pose.position
        print(f'Position: x={position.x}, y={position.y}, z={position.z}')
        file.write(f'{position.x} {position.y} {position.z}\n')

if __name__ == '__main__':
    open('./imu_odom.txt', 'w').close()
    rospy.init_node('save_odom')
    rospy.Subscriber('/imu_odom', Odometry, callback)
    rospy.spin()