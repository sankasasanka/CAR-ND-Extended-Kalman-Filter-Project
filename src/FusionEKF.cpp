#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

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


  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */


  H_laser_<<1,0,0,0,
    		0,1,0,0;
  
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_<<	1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;
  
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
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;
    
    
    Eigen::VectorXd input;
   

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      
  	  
      input={measurement_pack.raw_measurements_};
      
      double phi = input[1];
      // Normalize phi to [-pi, pi]
      while (phi > M_PI)  phi -= 2.0 * M_PI;
      while (phi < -M_PI) phi += 2.0 * M_PI;
      
      ekf_.x_ <<input[0]*cos(phi),input[0]*sin(phi),input[2]*cos(phi),input[2]*sin(phi);
      

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
      input={measurement_pack.raw_measurements_};
      ekf_.x_ <<input[0],input[1],0,0;

    }
   
	
    // done initializing, no need to predict or update
    is_initialized_ = true;
    cout << "Initialization done"<<"\n";
    previous_timestamp_ = measurement_pack.timestamp_;
    return;
  }

  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

 
  
  float noise_ax = 12;
  float noise_ay = 12;
 
  
  //cout <<measurement_pack.timestamp_<<"\n"<<ekf_.previous_timestamp_<<"\n";
  
  double dt = (measurement_pack.timestamp_ - ekf_.previous_timestamp_) / 1000000.0;
  //cout << "Time Elapsed:"<<dt<<"\n";
  
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_<<1,0,dt,0,
           0,1,0,dt,
           0,0,1,0,
           0,0,0,1;

  // TODO: YOUR CODE HERE
  double dt_2 = dt * dt;
  double dt_3 = dt_2 * dt/2;
  double dt_4 = dt_3 * dt/4;

  // set the process covariance matrix Q
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ << dt_4*noise_ax, 0, dt_3*noise_ax, 0,
             0, dt_4*noise_ay, 0, dt_3*noise_ay,
             dt_3*noise_ax, 0, dt_2*noise_ax, 0,
             0, dt_3*noise_ay, 0, dt_2*noise_ay;
  //cout << "Q done"<<"\n";

  ekf_.Predict();
  //cout << "Prediction done"<<"\n";
 

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
    //cout << "Entered Radar Update"<<"\n";
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.R_ = R_radar_;
    
    
    //cout<<"radar Angle:"<<measurement_pack.raw_measurements_<<"\n";
  	ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    // TODO: Laser updates
    //cout << "Entered laser Update"<<"\n";
   	ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
  	ekf_.Update(measurement_pack.raw_measurements_);
	
  }
  ekf_.previous_timestamp_ = measurement_pack.timestamp_;
  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
