#include <eigen3/Eigen/Dense>

using namespace Eigen;

namespace imu_fusion
{
    class KalmanFilter
    {
    private:
        VectorXd State; // pos(x,y,z) and vel(x,y,z)
        VectorXd Z;     // measurement vector

        MatrixXd A; // state transition matrix
        MatrixXd H; // measurement matrix
        MatrixXd Q; // state noise matrix
        MatrixXd R; // measurement noise matrix
        MatrixXd P; // covariance matrix

        MatrixXd K; // kalman gain matrix

    public:
        KalmanFilter(double r = 0.5, double q = 0.1);
        void init(VectorXd state);
        void predict(double dt);
        void update(Vector3d z);
        Eigen::VectorXd getState();
    };
}