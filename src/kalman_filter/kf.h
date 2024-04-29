#include <Eigen/Dense>
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
        MatrixXd Q; // state convariance noise matrix
        MatrixXd R; // measurement convariance noise matrix

        MatrixXd K; // kalman gain matrix

    public:
        KalmanFilter()
        {
            State = VectorXd(6);
            Z = VectorXd(3);

            A = MatrixXd(6, 6);
            H = MatrixXd(3, 6); // measurement_dim , state_dim
            Q = MatrixXd(6, 6);
            R = MatrixXd(3, 3);

            A << 1, 0, 0, 1, 0, 0,
                0, 1, 0, 0, 1, 0,
                0, 0, 1, 0, 0, 1,
                0, 0, 0, 1, 0, 0,
                0, 0, 0, 0, 1, 0,
                0, 0, 0, 0, 0, 1;

            H << 0, 0, 0, 1, 0, 0,
                0, 0, 0, 0, 1, 0,
                0, 0, 0, 0, 0, 1;
        }
        void predict(double dt)
        {
            A(0, 3) = dt;
            A(1, 4) = dt;
            A(2, 5) = dt;
            State = A * State;
        }
        void update(VectorXd z)
        {
            Z = z;
            K = State * H.transpose() * (H * State * H.transpose() + R).inverse();
            State = State + K * (Z - H * State);
        }
    };
}