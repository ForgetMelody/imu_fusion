// ROS interface for the imu_preintegration package

#ifndef ROS_INTERFACE_H
#define ROS_INTERFACE_H

#include "ros/publisher.h"
#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <eigen3/Eigen/Dense>

namespace imu_fusion
{
    // IMU data types
    class IMUData
    {
    public:
        double time_stamp;
        Eigen::Vector3d acc;
        Eigen::Vector3d gyro;
        Eigen::Quaterniond orientation;
        void Print();
    };

    class OdomData
    {
    public:
        double time_stamp;
        Eigen::Vector3d pos;
        Eigen::Quaterniond orientation;
        Eigen::Vector3d linear_vel;
        Eigen::Vector3d angular_vel;
    };

    class INTERFACE
    {
    private:
        INTERFACE() = default;
        //nodehandle
        ros::NodeHandle *nh_local_ptr_;
        ros::NodeHandle *nh_ptr_;
        //subscriber
        ros::Subscriber imu_sub_;
        ros::Subscriber odom_sub_;
        //publisher
        ros::Publisher odom_pub_;
        ros::Publisher raw_path_pub_, filtered_path_pub_;

        bool is_initialized_ = false;
        void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);
        void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);

    public:
        std::vector<IMUData> imu_data_list;
        std::vector<OdomData> odom_data_list;
        // publish msg
        nav_msgs::Odometry odom_;
        nav_msgs::Path raw_path,fitten_path;

        IMUData get_imu_data();
        OdomData get_odom_data();
        //publish func
        void publish_odom();
        void publish_raw_path();
        void publish_fitten_path();
        //init
        void Init(int argc, char *argv[], std::string KNodeName);
        // get instance
        static INTERFACE &GetInterface()
        {
            static INTERFACE temp_;
            return temp_;
        }
    };
}

#endif // ROS_INTERFACE_H