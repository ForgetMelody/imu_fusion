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
#include <common/IMU.h>
#include <common/Odometry.h>

namespace imu_fusion
{
    class INTERFACE
    {
    private:
        INTERFACE() = default;
        // nodehandle
        ros::NodeHandle *nh_local_ptr_;
        ros::NodeHandle *nh_ptr_;
        // subscriber
        ros::Subscriber imu_sub_;
        ros::Subscriber odom_sub_;
        // publisher
        ros::Publisher odom_pub_;
        ros::Publisher raw_path_pub_, filtered_path_pub_;

        bool is_initialized_ = false;

        // callback func
        void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);
        void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
        // publish msg
        nav_msgs::Odometry odom_;
        nav_msgs::Path raw_path_, fitten_path_;

        // raw data list
        std::vector<IMU> imu_data_list;
        std::vector<Odom> odom_data_list;

    public:
        // param
        double kf_q, kf_r;

        IMU get_imu_data();
        Odom get_odom_data();
        // publish func
        void publish_odom();
        void publish_raw_path();
        void publish_fitten_path();
        void publish_msg();
        // set msg
        void set_odom(Eigen::Vector3d pos, Eigen::Quaterniond orientation, Eigen::Vector3d linear_vel, Eigen::Vector3d angular_val, std::string frame_id, double time_stamp);
        void add_path_point(Eigen::Vector3d pos, Eigen::Quaterniond orientation, std::string frame_id, double time_stamp);
        // init
        void Init(int argc, char *argv[], std::string KNodeName);
        void msg_init();
        void param_init();
        // get instance
        static INTERFACE &GetInterface()
        {
            static INTERFACE temp_;
            return temp_;
        }
    };
}

#endif // ROS_INTERFACE_H