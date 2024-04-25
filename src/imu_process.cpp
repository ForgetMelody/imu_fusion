#include "geometry_msgs/PoseStamped.h"
#include "ros_interface/ros_interface.h"
#include <thread>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
using namespace imu_fusion;
class IMUProcess
{
public:
    INTERFACE *interface_ptr;
    bool is_exit = false, is_init = false;
    double curr_time = -1, last_time = -1;
    IMUData imu;
    OdomData odom;
    Eigen::Matrix3d R_transform;
    //绕 z 轴转 - 90度 调整坐标系


    Eigen::Vector3d linear_vel;
    Eigen::Vector3d angular_vel;
    Eigen::Quaterniond R; // rotation
    Eigen::Vector3d p;    // position

    IMUProcess(int argc, char **argv)
    {
        INTERFACE interface = INTERFACE::GetInterface();
        interface.Init(argc, argv, "imu_process");
        interface_ptr = &interface;

        // start process thread
        std::thread process_thread(&IMUProcess::ProcessThread, this);
        process_thread.detach();
        ros::spin();
    }

    void Init()
    {
        ROS_INFO("Init");
        // 绕 z 轴转 - 90度 调整坐标系
        R_transform << 0, -1, 0, 1, 0, 0, 0, 0, 1;
        odom = interface_ptr->get_odom_data();
        curr_time = odom.time_stamp;
        linear_vel = odom.linear_vel;
        angular_vel = odom.angular_vel;
        R = odom.orientation;
        p = odom.pos;
        // path msg init
        interface_ptr->raw_path.header.frame_id = "odom";
        interface_ptr->fitten_path.header.frame_id = "odom";

        is_init = true;
    }

    void Update()
    {
        // get data
        while (odom.time_stamp < curr_time)
        {
            odom = interface_ptr->get_odom_data();
        }
        // ROS_INFO("odom time stamp: %f", odom.time_stamp);
        while (imu.time_stamp < odom.time_stamp)
        {
            imu = interface_ptr->get_imu_data();
        }
        // ROS_INFO("imu time stamp: %f", imu.time_stamp);
        last_time = curr_time;
        curr_time = imu.time_stamp;
        angular_vel = imu.gyro;
        linear_vel = odom.linear_vel;
        R = R_transform * imu.orientation;
        p += R * linear_vel * (curr_time - last_time);

        // publish odom
        interface_ptr->odom_.header.stamp = ros::Time(curr_time);
        interface_ptr->odom_.header.frame_id = "odom";
        interface_ptr->odom_.pose.pose.position.x = p.x();
        interface_ptr->odom_.pose.pose.position.y = p.y();
        interface_ptr->odom_.pose.pose.position.z = p.z();
        interface_ptr->odom_.twist.twist.linear.x = linear_vel.x();
        interface_ptr->odom_.twist.twist.linear.y = linear_vel.y();
        interface_ptr->odom_.twist.twist.linear.z = linear_vel.z();
        interface_ptr->odom_.twist.twist.angular.x = angular_vel.x();
        interface_ptr->odom_.twist.twist.angular.y = angular_vel.y();
        interface_ptr->odom_.twist.twist.angular.z = angular_vel.z();
        interface_ptr->odom_.pose.pose.orientation.w = R.w();
        interface_ptr->odom_.pose.pose.orientation.x = R.x();
        interface_ptr->odom_.pose.pose.orientation.y = R.y();
        interface_ptr->odom_.pose.pose.orientation.z = R.z();
        interface_ptr->publish_odom();

        //path
        geometry_msgs::PoseStamped pose;
        pose.pose = interface_ptr->odom_.pose.pose;
        pose.header = interface_ptr->odom_.header;
        interface_ptr->fitten_path.header.stamp = ros::Time(curr_time);
        interface_ptr->fitten_path.poses.push_back(pose);

        // publish path

        interface_ptr->publish_fitten_path();
        interface_ptr->publish_raw_path();
        // ROS_INFO("odom time stamp: %f", odom.time_stamp);
        // ROS_INFO("linear_vel: %f %f %f", odom.linear_vel.x(), odom.linear_vel.y(), odom.linear_vel.z());
        // ROS_INFO("angular_vel: %f %f %f", odom.angular_vel.x(), odom.angular_vel.y(), odom.angular_vel.z());
        // ROS_INFO("orientation: %f %f %f %f", odom.orientation.w(), odom.orientation.x(), odom.orientation.y(), odom.orientation.z());
        ROS_INFO("curr_time: %f", curr_time);
    }

    void ProcessThread()
    {
        if (!is_init)
            Init();
        while (!is_exit) // 循环读取imu数据
        {
            Update();
        }
    }

    ~IMUProcess() { is_exit = true; }
};

int main(int argc, char **argv)
{
    IMUProcess imu_process(argc, argv);
    return 0;
}