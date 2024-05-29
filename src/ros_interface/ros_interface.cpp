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

        is_initialized_ = true;
        // path msg init
        raw_path_.header.frame_id = "odom";
        fitten_path_.header.frame_id = "odom";

        ROS_INFO("ROS interface initialize finished.");
    }

    void INTERFACE::msg_init()
    {
        // subscriber
        imu_sub_ = nh_local_ptr_->subscribe<sensor_msgs::Imu>("/IMU_data", 10, boost::bind(&INTERFACE::imuCallback, this, _1));
        odom_sub_ = nh_local_ptr_->subscribe<nav_msgs::Odometry>("/odom", 10, boost::bind(&INTERFACE::odomCallback, this, _1));
        // publisher
        raw_path_pub_ = nh_local_ptr_->advertise<nav_msgs::Path>("/raw_path", 10);
        filtered_path_pub_ = nh_local_ptr_->advertise<nav_msgs::Path>("/filtered_path", 10);
        odom_pub_ = nh_local_ptr_->advertise<nav_msgs::Odometry>("/imu_odom", 10);
    }
    void INTERFACE::param_init()
    {
        // param
        nh_local_ptr_->param("kf_q", kf_q, 0.1);
        nh_local_ptr_->param("kf_r", kf_r, 0.1);
        ROS_INFO("Get Params: kf_q: %f, kf_r: %f", kf_q, kf_r);
    }
    void INTERFACE::imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
    {
        IMU imu_data;
        // time stamp to double
        imu_data.t_ = msg->header.stamp.toSec();
        // linear acceleration
        imu_data.acc_ << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
        // angular velocity
        imu_data.gyro_ << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;

        imu_data_list.push_back(imu_data);
    }
    void INTERFACE::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        Odom odom_data;
        // time stamp
        odom_data.t_ = msg->header.stamp.toSec();
        // pos
        odom_data.pos_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
        // orientation
        odom_data.quat_.x() = msg->pose.pose.orientation.x;
        odom_data.quat_.y() = msg->pose.pose.orientation.y;
        odom_data.quat_.z() = msg->pose.pose.orientation.z;
        odom_data.quat_.w() = msg->pose.pose.orientation.w;
        // linear vel
        odom_data.linear_ << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z; // z 一般为 0 (差速轮则x也为0)
        // angular vel
        odom_data.angular_ << msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z; // x y 一般为0

        odom_data_list.push_back(odom_data);
        // add to path
        geometry_msgs::PoseStamped pose;
        pose.pose = msg->pose.pose;
        raw_path_.header.stamp = msg->header.stamp;
        raw_path_.poses.push_back(pose);
    }

    void INTERFACE::publish_odom() { odom_pub_.publish(odom_); }
    void INTERFACE::publish_raw_path() { raw_path_pub_.publish(raw_path_); }
    void INTERFACE::publish_fitten_path() { filtered_path_pub_.publish(fitten_path_); }

    IMU INTERFACE::get_imu_data()
    {
        while (imu_data_list.empty())
            ros::Duration(0.02).sleep();
        IMU data = imu_data_list.front();
        imu_data_list.erase(imu_data_list.begin());
        return data;
    }
    Odom INTERFACE::get_odom_data()
    {
        while (odom_data_list.empty())
            ros::Duration(0.02).sleep();
        Odom data = odom_data_list.front();
        odom_data_list.erase(odom_data_list.begin());
        return data;
    }

    void INTERFACE::set_odom(Eigen::Vector3d pos, Eigen::Quaterniond orientation, Eigen::Vector3d linear_vel, Eigen::Vector3d angular_val, std::string frame_id, double time_stamp)
    {
        odom_.header.frame_id = frame_id;
        odom_.header.stamp = ros::Time(time_stamp);
        // pos
        odom_.pose.pose.position.x = pos.x();
        odom_.pose.pose.position.y = pos.y();
        odom_.pose.pose.position.z = pos.z();
        // orientation
        odom_.pose.pose.orientation.x = orientation.x();
        odom_.pose.pose.orientation.y = orientation.y();
        odom_.pose.pose.orientation.z = orientation.z();
        odom_.pose.pose.orientation.w = orientation.w();
        // val
        odom_.twist.twist.linear.x = linear_vel.x();
        odom_.twist.twist.linear.y = linear_vel.y();
        odom_.twist.twist.linear.z = linear_vel.z();
        odom_.twist.twist.angular.x = angular_val.x();
        odom_.twist.twist.angular.y = angular_val.y();
        odom_.twist.twist.angular.z = angular_val.z();
    }
    void INTERFACE::add_path_point(Eigen::Vector3d pos, Eigen::Quaterniond orientation, std::string frame_id, double time_stamp)
    {
        geometry_msgs::PoseStamped pose;
        // pos
        pose.pose.position.x = pos.x();
        pose.pose.position.y = pos.y();
        pose.pose.position.z = pos.z();
        // orientation
        pose.pose.orientation.x = orientation.x();
        pose.pose.orientation.y = orientation.y();
        pose.pose.orientation.z = orientation.z();
        pose.pose.orientation.w = orientation.w();
        // frame_id
        pose.header.frame_id = frame_id;
        // time_stamp
        pose.header.stamp = ros::Time(time_stamp);
        // add to path
        fitten_path_.header.stamp = ros::Time(time_stamp);
        fitten_path_.poses.push_back(pose);
    }
    void INTERFACE::publish_msg()
    {
        publish_odom();
        publish_raw_path();
        publish_fitten_path();
    }
}
// namespace imu_preintegration
