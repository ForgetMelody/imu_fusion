#include "ros_interface/ros_interface.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "ros/duration.h"

namespace imu_fusion
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
        //subscriber
        imu_sub_ = nh.subscribe<sensor_msgs::Imu>("/IMU_data", 1000, boost::bind(&INTERFACE::imuCallback, this, _1));
        odom_sub_ = nh.subscribe<nav_msgs::Odometry>("/odom", 1000, boost::bind(&INTERFACE::odomCallback, this, _1));
        //publisher
        raw_path_pub_ = nh.advertise<nav_msgs::Path>("/raw_path", 1000);
        filtered_path_pub_ = nh.advertise<nav_msgs::Path>("/filtered_path",1000);
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
        imu_data.orientation.x() = msg->orientation.x;
        imu_data.orientation.y() = msg->orientation.y;
        imu_data.orientation.z() = msg->orientation.z;
        imu_data.orientation.w() = msg->orientation.w;

        imu_data_list.push_back(imu_data);
    }
    void INTERFACE::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        OdomData odom_data;
        // time stamp
        odom_data.time_stamp = msg->header.stamp.toSec();
        // pos
        odom_data.pos << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
        // orientation
        odom_data.orientation.x() = msg->pose.pose.orientation.x;
        odom_data.orientation.y() = msg->pose.pose.orientation.y;
        odom_data.orientation.z() = msg->pose.pose.orientation.z;
        odom_data.orientation.w() = msg->pose.pose.orientation.w;
        // linear vel
        odom_data.linear_vel << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z; // z 一般为 0 (差速轮则x也为0)
        // angular vel
        odom_data.angular_vel << msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z; // x y 一般为0

        odom_data_list.push_back(odom_data);
        // add to path
        geometry_msgs::PoseStamped pose;
        pose.pose = msg->pose.pose;
        raw_path.header.stamp = msg->header.stamp;
        raw_path.poses.push_back(pose);
        
    }

    void INTERFACE::publish_odom(){odom_pub_.publish(odom_);}
    void INTERFACE::publish_raw_path(){raw_path_pub_.publish(raw_path);}
    void INTERFACE::publish_fitten_path(){filtered_path_pub_.publish(fitten_path);}

    IMUData INTERFACE::get_imu_data()
    {
        while (imu_data_list.empty())
            ros::Duration(0.02).sleep();
        IMUData data = imu_data_list.front();
        imu_data_list.erase(imu_data_list.begin());
        return data;
    }
    OdomData INTERFACE::get_odom_data()
    {
        while (odom_data_list.empty())
            ros::Duration(0.02).sleep();
        OdomData data = odom_data_list.front();
        odom_data_list.erase(odom_data_list.begin());
        return data;
    }
    void IMUData::Print()
    {
        std::cout << "IMUDATA: time_stamp: " << time_stamp << " \n \tacc: " << acc.transpose() << " \n\tgyro: " << gyro.transpose() << " \n\tquat: " << orientation.coeffs().transpose() << std::endl;
    }
}
// namespace imu_preintegration
