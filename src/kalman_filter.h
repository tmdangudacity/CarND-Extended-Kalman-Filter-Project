#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "Eigen/Dense"

class KalmanFilter
{
    public:

        // state vector
        Eigen::VectorXd x_;

        // state covariance matrix
        Eigen::MatrixXd P_;

        // state transition matrix
        Eigen::MatrixXd F_;

        // process covariance matrix
        Eigen::MatrixXd Q_;

        // measurement matrix laser
        Eigen::MatrixXd H_laser_;

        // measurement matrix radar (Jacobian)
        Eigen::MatrixXd H_radar_;

        // measurement covariance matrix laser
        Eigen::MatrixXd R_laser_;

        // measurement covariance matrix laser
        Eigen::MatrixXd R_radar_;

        /**
        * Constructor
        */
        KalmanFilter();

        /**
        * Destructor
        */
        virtual ~KalmanFilter();

        /**
        * Init Initializes Kalman filter
        * @param x_in Initial state
        * @param P_in Initial state covariance
        * @param F_in Transition matrix
        * @param Q_in Process covariance matrix
        * @param H_laser_in Measurement matrix
        * @param H_radar_in Measurement matrix
        * @param R_laser_in Measurement covariance matrix
        * @param R_radar_in Measurement covariance matrix
        */
        void Init(const Eigen::VectorXd &x_in,
                  const Eigen::MatrixXd &P_in,
                  const Eigen::MatrixXd &F_in,
                  const Eigen::MatrixXd &Q_in,
                  const Eigen::MatrixXd &H_laser_in,
                  const Eigen::MatrixXd &H_radar_in,
                  const Eigen::MatrixXd &R_laser_in,
                  const Eigen::MatrixXd &R_radar_in);

        /**
        * Prediction Predicts the state and the state covariance
        * using the process model
        * @param delta_T Time between k and k+1 in s
        */
        void Predict();

        /**
        * Updates the state by using standard Kalman Filter equations
        * @param z The measurement at k+1
        */
        void Update(const Eigen::VectorXd &z);

        /**
        * Updates the state by using Extended Kalman Filter equations
        * @param z The measurement at k+1
        */
        void UpdateEKF(const Eigen::VectorXd &z);

    private:

        //Identity matrix
        Eigen::MatrixXd I_;




};

#endif /* KALMAN_FILTER_H_ */

