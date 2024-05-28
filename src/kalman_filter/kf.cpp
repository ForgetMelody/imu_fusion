#include <cstdio>
#include <iostream>
#include <kalman_filter/kf.h>

using namespace std;
using namespace Eigen;

namespace imu_fusion
{

    KalmanFilter::KalmanFilter(double r, double q)
    {
        State = VectorXd(6); // 状态向量
        Z = VectorXd(3);     // 测量值

        A = MatrixXd(6, 6);
        H = MatrixXd(3, 6); // measurement_dim , state_dim
        Q = MatrixXd(6, 6);
        R = MatrixXd(3, 3);
        P = MatrixXd::Identity(6, 6);

        A << 1, 0, 0, 1, 0, 0,
            0, 1, 0, 0, 1, 0,
            0, 0, 1, 0, 0, 1,
            0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 1;

        H << 0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 1;
        // noise

        Q << q, 0, 0, 0, 0, 0,
            0, q, 0, 0, 0, 0,
            0, 0, q, 0, 0, 0,
            0, 0, 0, q, 0, 0,
            0, 0, 0, 0, q, 0,
            0, 0, 0, 0, 0, q;

        R << r, 0, 0,
            0, r, 0,
            0, 0, r;
    }
    void KalmanFilter::init(VectorXd state)
    {
        State = state;
    }

    void KalmanFilter::predict(double dt)
    {
        // predict state
        A(0, 3) = dt;
        A(1, 4) = dt;
        A(2, 5) = dt;
        State = A * State;
        // predict covariance
        // P = A * P * A.transpose() + Q;
        P = A * P * A.transpose() + Q;
    }
    void KalmanFilter::update(Vector3d z)
    {
        Z << z(0), z(1), z(2);

        // update kalman gain
        K = Q * H.transpose() * (H * Q * H.transpose() + R).inverse();
        // update state estimate
        State = State + K * (Z - H * State);
        // update covariance matrix
        P = (MatrixXd::Identity(6, 6) - K * H) * P;
    }
    Eigen::VectorXd KalmanFilter::getState()
    {
        return State;
    }
}
