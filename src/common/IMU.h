#ifndef IMU_H
#define IMU_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
struct IMU
{
    IMU() = default;
    IMU(double t,
        const Eigen::Vector3d &gyro,
        const Eigen::Vector3d &acc) : gyro_(gyro), acc_(acc), t_(t) {}

    double t_;
    Eigen::Vector3d gyro_;
    Eigen::Vector3d acc_;
};
#endif // IMU_H