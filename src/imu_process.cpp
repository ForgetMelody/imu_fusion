#include "ros_interface/ros_interface.h"
#include "kalman_filter/kf.h"
#include <thread>
#include <ros/ros.h>
using namespace imu_fusion;
class IMUProcess
{
public:
    INTERFACE *interface_ptr;
    bool is_exit = false, is_init = false;
    double curr_time = -1, last_time = -1;

    IMU imu;
    Odom odom;
    KalmanFilter *kf;
    Eigen::Vector3d linear_vel;
    Eigen::Vector3d angular_vel;
    Eigen::Quaterniond R; // rotation
    Eigen::Vector3d P;    // position
    Eigen::Vector3d G_bias;

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
        odom = interface_ptr->get_odom_data();
        imu = interface_ptr->get_imu_data();
        curr_time = odom.t_;
        linear_vel = odom.linear_;
        angular_vel = odom.angular_;
        R = Eigen::Quaterniond::Identity();

        for (int i = 0; i < 50; i++)
        {
            // record bias of gyro
            odom = interface_ptr->get_odom_data();
            imu = interface_ptr->get_imu_data();
            curr_time = odom.t_;
            G_bias += imu.gyro_;
        }
        G_bias /= 50;
        ROS_INFO("G_bias: %f %f %f", G_bias.x(), G_bias.y(), G_bias.z());

        Vector3d rpy = R.toRotationMatrix().eulerAngles(0, 1, 2);
        VectorXd state(6);
        state << rpy.x(), rpy.y(), rpy.z(), angular_vel.x(), angular_vel.y(), angular_vel.z();
        kf = new KalmanFilter(interface_ptr->kf_r, interface_ptr->kf_q);
        kf->init(state);
        P = odom.pos_;
        is_init = true;
    }

    void Update()
    {
        // get data
        // while (odom.time_stamp < curr_time)
        odom = interface_ptr->get_odom_data();
        // ROS_INFO("odom time stamp: %f", odom.time_stamp);
        // while (imu.time_stamp < odom.time_stamp)
        imu = interface_ptr->get_imu_data();
        // ROS_INFO("imu time stamp: %f", imu.time_stamp);

        // dt
        last_time = curr_time;
        curr_time = imu.t_;
        double dt = curr_time - last_time;

        // kalman filter predict and update
        kf->predict(dt);
        Vector3d z_gyro(imu.gyro_.x() - G_bias.x(), imu.gyro_.y() - G_bias.y(), imu.gyro_.z() - G_bias.z());
        kf->update(z_gyro);

        Quaterniond R((Eigen::AngleAxisd(kf->getState()[0], Eigen::Vector3d::UnitX()) *
                       Eigen::AngleAxisd(kf->getState()[1], Eigen::Vector3d::UnitY()) *
                       Eigen::AngleAxisd(kf->getState()[2], Eigen::Vector3d::UnitZ()))
                          .toRotationMatrix());

        linear_vel = odom.linear_;
        angular_vel = kf->getState().tail(3);
        printf("linear_vel: \t%f \t%f \t%f angular_vel: \t%f \t%f \t%f\n", linear_vel.x(), linear_vel.y(), linear_vel.z(), angular_vel.x(), angular_vel.y(), angular_vel.z());

        P += R * linear_vel * (curr_time - last_time);

        // publish odom
        interface_ptr->set_odom(P, R, linear_vel, angular_vel, "odom", curr_time);
        // path
        interface_ptr->add_path_point(P, R, "odom", curr_time);
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
            Update();
    }

    ~IMUProcess() { is_exit = true; }
};

int main(int argc, char **argv)
{
    IMUProcess imu_process(argc, argv);
    return 0;
}