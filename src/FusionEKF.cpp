#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

const float FusionEKF::NOISE_AX = 9.0;
const float FusionEKF::NOISE_AY = 9.0;

//Constructor
FusionEKF::FusionEKF(DataOption in_data)
:data_option_(in_data),
 is_initialized_(false),
 previous_timestamp_(0.0)
{
    //initializing matrices

    //state covariance matrix
    P_ = MatrixXd(4, 4);
    P_ <<   1.0, 0.0,    0.0,    0.0,
            0.0, 1.0,    0.0,    0.0,
            0.0, 0.0, 1000.0,    0.0,
            0.0, 0.0,    0.0, 1000.0;

    //measurement covariance matrix - laser
    R_laser_ = MatrixXd(2, 2);
    R_laser_ << 0.0225, 0.0,
                0.0, 0.0225;

    //measurement covariance matrix - radar
    R_radar_ = MatrixXd(3, 3);
    R_radar_ << 0.09, 0.0, 0.0,
                0.0, 0.0009, 0.0,
                0.0, 0.0, 0.09;

    //laser measurement matrix
    H_laser_ = MatrixXd(2, 4);
    H_laser_ << 1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0;

    //radar measurement matrix
    H_radar_ = MatrixXd(3, 4);
    H_radar_ << 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0;

    //transition matrix
    F_ = MatrixXd(4, 4);
    F_ << 1.0, 0.0, 1.0, 0.0,
          0.0, 1.0, 0.0, 1.0,
          0.0, 0.0, 1.0, 0.0,
          0.0, 0.0, 0.0, 1.0;

    //process covariance matrix

    Q_ = MatrixXd(4, 4);
    Q_ << 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0;
}

//Destructor.
FusionEKF::~FusionEKF() {}

bool FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack)
{
    bool ret = false;

    //If not initialised yet
    if (!is_initialized_)
    {
        //Initialize the state ekf_.x_ with the first measurement.
        //Create the covariance matrix.
        //Remember: you'll need to convert radar from polar to cartesian coordinates.

        // first measurement
        VectorXd x_init = VectorXd(4);

        if(measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
        {
            //Convert radar from polar to cartesian coordinates and initialize state.
            float ro     = measurement_pack.raw_measurements_(0);
            float theta  = Tools::UnwrapAngle(measurement_pack.raw_measurements_(1));
            float ro_dot = measurement_pack.raw_measurements_(2);

            x_init(0) = ro * cos(theta);
            x_init(1) = ro * sin(theta);
            x_init(2) = ro_dot * cos(theta);
            x_init(3) = ro_dot * sin(theta);

            is_initialized_ = true;

            std::cout << "Kalman Filter initialised with Radar data:" << std::endl;
            std::cout << x_init << std::endl;
        }
        else if(measurement_pack.sensor_type_ == MeasurementPackage::LASER)
        {
            //Initialize state.
            x_init(0) = measurement_pack.raw_measurements_(0);
            x_init(1) = measurement_pack.raw_measurements_(1);
            x_init(2) = 1.0;
            x_init(3) = 1.0;

            std::cout << "Kalman Filter initialised with Laser data:" << std::endl;
            std::cout << x_init << std::endl;
        }

        is_initialized_ = true;
        ekf_.Init(x_init, P_, F_, Q_, H_laser_, H_radar_, R_laser_, R_radar_);
        previous_timestamp_ = measurement_pack.timestamp_;

        std::cout << "Fusion data option: " << ToString(data_option_) << std::endl;

        ret = is_initialized_;
    }
    else
    {
        //If already initialised:

        //Prediction step
        ret = RunPrediction(measurement_pack.timestamp_);

        //Update step:
        if(ret)
        {
            if((data_option_ != LASER_DATA_ONLY) && (measurement_pack.sensor_type_ == MeasurementPackage::RADAR))
            {
                //Radar updates
                bool j_status = false;

                ekf_.H_radar_ = tools.CalculateJacobian(ekf_.x_, &j_status);
                if(j_status)
                {
                    std::cout << "Update with Radar measurement" << std::endl;
                    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
                }
                else
                {
                    std::cout << "Update Radar Measurement, Calculation of Jacobian FAILED!" << std::endl;
                }
            }
            else if((data_option_ != RADAR_DATA_ONLY) && (measurement_pack.sensor_type_ == MeasurementPackage::LASER))
            {
                //Laser updates
                std::cout << "Update with Laser measurement" << std::endl;
                ekf_.Update(measurement_pack.raw_measurements_);
            }
            //Otherwise not update because of wrong data option.

            std::cout << "x_ =" << std::endl;
            std::cout << ekf_.x_ << std::endl;

            std::cout << "P_ =" << std::endl;
            std::cout << ekf_.P_ << std::endl;
        }
    }

    return ret;
}

bool FusionEKF::RunPrediction(long long timestamp)
{
    //Time is measured in seconds.
    //Update the state transition matrix F according to the new elapsed time.
    //Update the process noise covariance matrix.
    //Use noise_ax = 9 and noise_ay = 9 for your Q matrix.

    bool ret = (timestamp > previous_timestamp_);

    if(ret)
    {
        float dt = 1.0e-6 * (timestamp - previous_timestamp_);
        previous_timestamp_ = timestamp;

        std::cout << "Run Prediction, dt: " << dt << std::endl;

        float dt_2 = dt * dt;
        float dt_3 = dt_2 * dt;
        float dt_4 = dt_3 * dt;

        //Prediction step:
        ekf_.F_(0, 2) = dt;
        ekf_.F_(1, 3) = dt;

        ekf_.Q_ <<  (0.25 * dt_4 * NOISE_AX), 0.0, (0.5 * dt_3 * NOISE_AX), 0.0,
                    0.0, (0.25 * dt_4 * NOISE_AY), 0.0, (0.5 * dt_3 * NOISE_AY),
                    (0.5 * dt_3 * NOISE_AX), 0.0, (dt_2 * NOISE_AX), 0.0,
                    0.0, (0.5 * dt_3 * NOISE_AY), 0.0, (dt_2 * NOISE_AY);

        ekf_.Predict();
    }
    else
    {
        std::cout << "Run Prediction ERROR!"
                  << " Current measurement timestamp " << timestamp
                  << " <= Previous timestamp "         << previous_timestamp_
                  << std::endl;
    }

    return ret;
}

std::string FusionEKF::ToString(DataOption data_option)
{
    std::string ret = "UNKNOWN_DATA_OPTION";

    switch(data_option)
    {
        case LASER_DATA_ONLY:
            ret = "LASER DATA ONLY";
            break;

        case RADAR_DATA_ONLY:
            ret = "RADAR DATA ONLY";
            break;

        case LASER_AND_RADAR_DATA:
            ret = "LASER AND RADAR DATA";
            break;

        default:
            break;
    }

    return ret;
}

