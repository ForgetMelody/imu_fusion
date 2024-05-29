#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
struct Odom
{
    Odom() = default;
    Odom(double t,
         const Eigen::Vector3d &pos,
         const Eigen::Quaterniond &quat,
         const Eigen::Vector3d &linear,
         const Eigen::Vector3d &angular) : t_(t), pos_(pos), quat_(quat), linear_(linear), angular_(angular) {}

    double t_;
    Eigen::Vector3d pos_;
    Eigen::Quaterniond quat_;
    Eigen::Vector3d linear_;
    Eigen::Vector3d angular_;
};
#endif // ODOMETRY_H