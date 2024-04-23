#include "ros_interface/ros_interface.h"
#include "ros/duration.h"

namespace imu_preintegration
{
    void INTERFACE::Init(int argc, char *argv[], std::string KNodeName)
    {
        if (is_initialized_)
        {
            ROS_WARN("ROS interface has already been initialized.");
            return;
        }
        ros::init(argc, argv, KNodeName);
        ros::NodeHandle nh;
        ros::NodeHandle nh_local("~");
        nh_ptr_ = &nh;
        nh_local_ptr_ = &nh_local;
        imu_sub_ = nh.subscribe<sensor_msgs::Imu>("/IMU_data", 1000, boost::bind(&INTERFACE::imuCallback, this, _1));
        odom_pub_ = nh.advertise<nav_msgs::Odometry>("/imu_odom", 1000);

        is_initialized_ = true;
        ROS_INFO("ROS interface initialize finished.");
    }

    void INTERFACE::imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
    {
        IMUData imu_data;
        // time stamp to double
        imu_data.time_stamp = msg->header.stamp.toSec();
        // linear acceleration
        imu_data.acc << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
        // angular velocity
        imu_data.gyro << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
        // orientation
        imu_data.quat.x() = msg->orientation.x;
        imu_data.quat.y() = msg->orientation.y;
        imu_data.quat.z() = msg->orientation.z;
        imu_data.quat.w() = msg->orientation.w;

        imu_data_list.push_back(imu_data);
    }

    void INTERFACE::publish_odom()
    {
        // Publish the odometry message
        odom_pub_.publish(odom_);
    }

    IMUData INTERFACE::getData() {
        while (imu_data_list.empty())
            ros::Duration(0.02).sleep();
        IMUData data = imu_data_list.front();
        imu_data_list.erase(imu_data_list.begin());
        return data;
    }
    void IMUData::Print() {
    std::cout <<"IMUDATA: time_stamp: " << time_stamp << " \n \tacc: " << acc.transpose() << " \n\tgyro: " << gyro.transpose() << " \n\tquat: " << quat.coeffs().transpose() << std::endl;
    }
}
// namespace imu_preintegration
