#include "kalman_filter.h"
#include<iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
  previous_timestamp_ = 0.0;
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
 
  
  x_ = F_ * x_;

  MatrixXd Ft = F_.transpose();
  
  MatrixXd P1 = P_*Ft;
  P_ = (F_*P1)  + Q_;

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
      VectorXd z_pred = H_ * x_;
      VectorXd y = z - z_pred;
      MatrixXd Ht = H_.transpose();
      MatrixXd S = H_ * (P_ * Ht) + R_;
      MatrixXd Si = S.inverse();
      MatrixXd PHt = P_ * Ht;
      MatrixXd K = PHt * Si;
      //std::cout<<"Update KF Done"<<"\n";
    
      //new estimate
      x_ = x_ + (K * y);
      long x_size = x_.size();
      MatrixXd I = MatrixXd::Identity(x_size, x_size);
      P_ = (I - K * H_) * P_;
  
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
    
      VectorXd h_x(3);
  	  h_x<<sqrt(x_[0]*x_[0]+x_[1]*x_[1]),
  		   atan2(x_[1],x_[0]),
  		   (x_[0]*x_[2]+x_[1]*x_[3])/sqrt(x_[0]*x_[0]+x_[1]*x_[1]);
  
  	// Normalizing the angles of the measurement data
  	  VectorXd z_cor(3);
  	  z_cor = z;
  
      if(z_cor(1) > M_PI ||z_cor(1) < -M_PI) {
              while (z_cor(1) > M_PI) {
                  z_cor(1) = z_cor(1) - 2 * M_PI;
              }
              while (z_cor(1) < -M_PI) {
                  z_cor(1) = z_cor(1)+2 * M_PI;
              }
          }
  
  
  	  //std::cout<<"x_[0]:"<<x_[0]<<"\tx_[1]:"<<x_[0]<<"\n";
  	  //std::cout<<"z:"<<z<<"\n";
      //std::cout<<"h_x:"<<h_x<<"\n";
  
      VectorXd z_pred = h_x; // Check 
      VectorXd y = z_cor - z_pred;
      MatrixXd Ht = H_.transpose();
      MatrixXd S = H_ * P_ * Ht + R_;
      MatrixXd Si = S.inverse();
      MatrixXd PHt = P_ * Ht;
      MatrixXd K = PHt * Si;
      //std::cout<<"Update EKF Done"<<"\n";
    
      //new estimate
      x_ = x_ + (K * y);
      long x_size = x_.size();
      MatrixXd I = MatrixXd::Identity(x_size, x_size);
      P_ = (I - K * H_) * P_;
    
}
