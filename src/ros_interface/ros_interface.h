// ROS interface for the imu_preintegration package

#ifndef ROS_INTERFACE_H
#define ROS_INTERFACE_H

#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Dense>

namespace imu_preintegration
{
    // IMU data types
    class IMUData
    {
    public:
        double time_stamp;
        Eigen::Vector3d acc;
        Eigen::Vector3d gyro;
        Eigen::Quaterniond quat;
        void Print();
        
    };
    class INTERFACE
    {
    private:
        INTERFACE() = default;
        ros::NodeHandle *nh_local_ptr_;
        ros::NodeHandle *nh_ptr_;
        ros::Subscriber imu_sub_;
        ros::Publisher odom_pub_;
        nav_msgs::Odometry odom_;

        bool is_initialized_ = false;
        void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);

    public:
        std::vector<IMUData> imu_data_list;
        void publish_odom();
        IMUData getData();
        void Init(int argc, char *argv[], std::string KNodeName);
        static INTERFACE &GetInterface()
        {
            static INTERFACE temp_;
            return temp_;
        }
    };
}

#endif // ROS_INTERFACE_H