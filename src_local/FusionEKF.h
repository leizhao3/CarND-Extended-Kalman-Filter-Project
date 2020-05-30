#ifndef FusionEKF_H_
#define FusionEKF_H_

#include <fstream>
#include <string>
#include <vector>
#include "Eigen/Dense"
#include "kalman_filter.h"
#include "measurement_package.h"
#include "tools.h"

class FusionEKF {
 public:
  /**
   * Constructor.
   */
  FusionEKF();

  /**
   * Destructor.
   */
  virtual ~FusionEKF();

  /**
   * Run the whole flow of the Kalman Filter from here.
   */
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

  /**
   * Kalman Filter update and prediction math lives in here.
  */
  KalmanFilter ekf_;

  private:
    // check whether the tracking toolbox was initialized or not (first measurement)
    bool is_initialized_;

    // previous timestamp
    long long previous_timestamp_;

    // tool object used to compute Jacobian and RMSE
    Tools tools;
    // hardcoded values
    Eigen::MatrixXd R_laser_;
    Eigen::MatrixXd R_radar_;
    Eigen::MatrixXd H_laser_;
    Eigen::MatrixXd Hj_;
    float sigma_ax2;
    float sigma_ay2;
};

#endif // FusionEKF_H_


/*--------------------------start of cpp----------------------------------*/

#include <iostream>
#include "Eigen/Dense"
#include "tools.h"
#include <math.h>
#include <iomanip> 
#include <ios> 

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  //** initializing matrices
  //measurement covariance matrix - laser
  R_laser_ = MatrixXd(2, 2);
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ = MatrixXd(3, 3);
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
  */
  // laser measurement matrix
  H_laser_ = MatrixXd(2, 4);
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  // radar measurement matrix
  Hj_ = MatrixXd(3, 4);


  // state covariance matrix P_
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_<< 1, 0, 0,    0,
            0, 1, 0,    0,
            0, 0, 1000, 0,
            0, 0, 0,    1000;

  // the initial transition matrix F_
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ = MatrixXd::Identity(4,4);

  // set the acceleration noise components, sigma_ax2, sigma_ay2
  sigma_ax2 = 9.f;
  sigma_ay2 = 9.f;
  }

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
      /**
       * TODO: Initialize the state ekf_.x_ with the first measurement.
       * TODO: Create the covariance matrix.
       * You'll need to convert radar from polar to cartesian coordinates.
      */

      // first measurement
      cout << "EKF is initializing...." << endl;

      if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
        // TODO: Initialize state.
        ekf_.x_ = MatrixXd(4, 1);
        ekf_.x_<< measurement_pack.raw_measurements_[0],
                  measurement_pack.raw_measurements_[1],
                  0,
                  0;
      } else if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        // TODO: Convert radar from polar to cartesian coordinates
        //         and initialize state.
        ekf_.x_ = MatrixXd(4, 1);
        float rho = measurement_pack.raw_measurements_[0]; 
        float phi = measurement_pack.raw_measurements_[1]; //unit: rad
        ekf_.x_<< rho*cos(phi),
                  phi*sin(phi),
                  0,
                  0;
      }

      previous_timestamp_ = measurement_pack.timestamp_;

      // done initializing, no need to predict or update
      is_initialized_ = true;
      return;
    }
    

    /**
     * TODO: Update the state transition matrix F according to the new elapsed time.
     * Time is measured in seconds.
     * TODO: Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
     */
    const float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
    previous_timestamp_ = measurement_pack.timestamp_;
    const float dt_2 = dt * dt;
    const float dt_3 = dt_2 * dt;
    const float dt_4 = dt_3 * dt;

    // Modify the F matrix so that the time is integrated
    ekf_.F_(0, 2) = dt;
    ekf_.F_(1, 3) = dt;

    // set the process covariance matrix Q
    ekf_.Q_ = MatrixXd(4, 4);
    ekf_.Q_ <<  dt_4/4*sigma_ax2,   0,                  dt_3/2*sigma_ax2,   0,
                0,                  dt_4/4*sigma_ay2,   0,                  dt_3/2*sigma_ay2,
                dt_3/2*sigma_ax2,   0,                  dt_2*sigma_ax2,     0,
                0,                  dt_3/2*sigma_ay2,   0,                  dt_2*sigma_ay2;


    //ekf_.Init(x_, P_, F_, H_, R_, Q_);

    /**
    * Prediction
    */
    ekf_.Predict();

    /**
     * Update
    */

    /**
     * TODO:
     * - Use the sensor type to perform the update step.
     * - Update the state and covariance matrices.
    */

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Radar updates
      ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
      ekf_.R_ = R_radar_;
      ekf_.UpdateEKF(measurement_pack.raw_measurements_);

    } else {
      // TODO: Laser updates
      ekf_.H_ = H_laser_;
      ekf_.R_ = R_laser_;
      ekf_.Update(measurement_pack.raw_measurements_);

    }

    // print the output
    cout << "x_ = " << endl;
    for(int i=0; i<4; i++)
    {
       cout << ekf_.x_[i] << endl;
    }

    cout << "P_ = " << endl;
    const int width = 15;
    for(int i=0; i<4; i++)
    {
      for(int j=0; j<4; j++)
      {
        cout <<std::setw(width)<< ekf_.P_(i,j);
      }
      cout << endl;
    }
    cout << endl;

}

