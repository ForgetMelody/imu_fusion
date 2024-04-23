#include "ros_interface/ros_interface.h"
#include <thread>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
using namespace imu_preintegration;
class IMUProcess
{
public:
    INTERFACE *interface_ptr;
    bool is_exit = false, is_init = false;
    double curr_time = -1, last_time = -1;
    Eigen::Vector3d static_acc,static_gyro;
    Eigen::Vector3d gravity;
    Eigen::Quaterniond R; // rotation
    Eigen::Vector3d v;    // velocity
    Eigen::Vector3d p;    // position

    IMUProcess(int argc, char **argv)
    {
        INTERFACE interface = INTERFACE::GetInterface();
        interface.Init(argc, argv, "imu_process");
        interface_ptr = &interface;
        // init variables
        R.setIdentity();
        v.setZero();
        p.setZero();
        gravity << 0, 0, 9.8;
        // start process thread
        std::thread process_thread(&IMUProcess::ProcessThread, this);
        process_thread.detach();
        ros::spin();
    }
    
    void Init()
    {
        // 假定前20次数据是静止的，使用这部分数据进行初始化
        // calculate gravity
        int i = 0,k = 0;
        IMUData imu_data;
        while (i < 100)
        {
            imu_data = interface_ptr->getData();
            imu_data.Print();
            if (imu_data.gyro.norm() < 0.01) {
                static_acc += imu_data.acc;
                k++;
            } else {
                ROS_WARN("gyro is not zero(%lf,%lf,%lf), skip this data", imu_data.gyro(0), imu_data.gyro(1), imu_data.gyro(2));
            }
            static_gyro += imu_data.gyro;
            i++; 
        }
        static_acc/= k;
        static_gyro/= i;
        ROS_INFO("Average_static_status: %lf %lf %lf", static_acc(0), static_acc(1), static_acc(2));
        ROS_INFO("Average_static_gyro: %lf %lf %lf", static_gyro(0), static_gyro(1), static_gyro(2));
        ROS_INFO("Gravity: %lf %lf %lf", gravity(0), gravity(1), gravity(2));
        // 初始化imu数据
        is_init = true;
        // 更新时间
        last_time = curr_time;
        curr_time = imu_data.time_stamp;
    }

    void Update()
    {
        //get data
        IMUData data = interface_ptr->getData();
       // 更新时间
        last_time = curr_time;
        curr_time = data.time_stamp;
        double dt = curr_time - last_time;
    }
    void ProcessThread()
    {
        while (!is_exit)// 循环读取imu数据
        {
            if (!is_init) // 第一次读取数据进行初始化
                Init();
            else
                Update();
        }
    }
    ~IMUProcess()
    {
        is_exit = true;
    }
};

int main(int argc, char **argv)
{
    IMUProcess imu_process(argc, argv);
    return 0;
}