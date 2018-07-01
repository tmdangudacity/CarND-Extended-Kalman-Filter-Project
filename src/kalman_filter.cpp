#include "kalman_filter.h"
#include "tools.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter()
{
}

KalmanFilter::~KalmanFilter()
{
}

void KalmanFilter::Init(const Eigen::VectorXd &x_in,
                        const Eigen::MatrixXd &P_in,
                        const Eigen::MatrixXd &F_in,
                        const Eigen::MatrixXd &Q_in,
                        const Eigen::MatrixXd &H_laser_in,
                        const Eigen::MatrixXd &H_radar_in,
                        const Eigen::MatrixXd &R_laser_in,
                        const Eigen::MatrixXd &R_radar_in)
{
    x_ = x_in;

    P_ = P_in;
    F_ = F_in;
    Q_ = Q_in;

    H_laser_ = H_laser_in;
    H_radar_ = H_radar_in;
    R_laser_ = R_laser_in;
    R_radar_ = R_radar_in;

    long x_size = x_.size();
    I_ = MatrixXd::Identity(x_size, x_size);
}

void KalmanFilter::Predict()
{
    x_ = F_ * x_;

    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

//Update with Laser data
void KalmanFilter::Update(const VectorXd &z)
{
    VectorXd z_predict = H_laser_ * x_;
    VectorXd y         = z - z_predict;
    MatrixXd Ht        = H_laser_.transpose();
    MatrixXd S         = H_laser_ * P_ * Ht + R_laser_;
    MatrixXd Si        = S.inverse();
    MatrixXd K         = P_ * Ht * Si;

    //new estimate
    x_ = x_ + (K * y);
    P_ = (I_ - K * H_laser_) * P_;
}

//Update with Radar data
void KalmanFilter::UpdateEKF(const VectorXd &z)
{
    //update the state by using Extended Kalman Filter equations
    float px = x_(0);
    float py = x_(1);
    float ro = sqrt(px * px + py * py);

    if(ro > 0.0)
    {
        float vx     = x_(2);
        float vy     = x_(3);
        float theta  = atan2(py, px);
        float ro_dot = (px * vx + py * vy) / ro;

        VectorXd z_predict = VectorXd(3);
        z_predict(0) = ro;
        z_predict(1) = theta;
        z_predict(2) = ro_dot;

        VectorXd y  = z - z_predict;
        y(1) = Tools::UnwrapAngle(y(1));

        MatrixXd Ht = H_radar_.transpose();
        MatrixXd S  = H_radar_ * P_ * Ht + R_radar_;
        MatrixXd Si = S.inverse();

        MatrixXd K  = P_ * Ht * Si;

        //new estimate
        x_ = x_ + (K * y);
        P_ = (I_ - K * H_radar_) * P_;
    }
    else
    {
        std::cout << "ERROR! UpdateEKF Ro: " << ro << std::endl;
    }
}

