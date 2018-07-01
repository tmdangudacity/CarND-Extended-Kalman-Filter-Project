#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"
#include "tools.h"

class FusionEKF
{
    public:

        typedef enum
        {
            LASER_DATA_ONLY = 0,
            RADAR_DATA_ONLY,
            LASER_AND_RADAR_DATA
        }
        DataOption;

        /**
        * Constructor.
        */
        FusionEKF(DataOption in_data = LASER_AND_RADAR_DATA);

        /**
        * Destructor.
        */
        virtual ~FusionEKF();

        /**
        * Run the whole flow of the Kalman Filter from here.
        */
        bool ProcessMeasurement(const MeasurementPackage &measurement_pack);

        /**
        * Kalman Filter update and prediction math lives in here.
        */
        KalmanFilter ekf_;

        static std::string ToString(DataOption data_option);

    private:

        // what stream of data is used
        DataOption data_option_;

        // check whether the tracking toolbox was initialized or not (first measurement)
        bool is_initialized_;

        // previous timestamp
        long long previous_timestamp_;

        bool RunPrediction(long long timestamp);

        // tool object used to compute Jacobian and RMSE
        Tools tools;

        Eigen::MatrixXd P_;
        Eigen::MatrixXd F_;
        Eigen::MatrixXd Q_;

        Eigen::MatrixXd H_laser_;
        Eigen::MatrixXd H_radar_;

        Eigen::MatrixXd R_laser_;
        Eigen::MatrixXd R_radar_;

        static const float NOISE_AX;
        static const float NOISE_AY;
};

#endif /* FusionEKF_H_ */
