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
    // 绕 z 轴转 - 90度 调整坐标系

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

        is_init = true;
    }

    void Update()
    {
        // get data
        while (odom.time_stamp < curr_time)
            odom = interface_ptr->get_odom_data();

        // ROS_INFO("odom time stamp: %f", odom.time_stamp);

        while (imu.time_stamp < odom.time_stamp)
            imu = interface_ptr->get_imu_data();

        // ROS_INFO("imu time stamp: %f", imu.time_stamp);

        last_time = curr_time;
        curr_time = imu.time_stamp;
        angular_vel = imu.gyro;
        linear_vel = odom.linear_vel;
        R = R_transform * imu.orientation;
        p += R * linear_vel * (curr_time - last_time);

        // publish odom
        interface_ptr->set_odom(p, R, linear_vel, angular_vel, "odom", curr_time);
        // path
        interface_ptr->add_path_point(p, R, "odom", curr_time);
        // publish path
        interface_ptr->publish_msg();
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