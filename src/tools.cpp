#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if (estimations.size() != ground_truth.size()
      || estimations.size() == 0) {
    std::cout << "Invalid estimation or ground_truth data" << std::endl;
    return rmse;
  }

  // accumulate squared residuals
  for (unsigned int i=0; i < estimations.size(); ++i) {

    VectorXd residual = estimations[i] - ground_truth[i];

    // coefficient-wise multiplication
    residual = residual.array()*residual.array();
    rmse += residual;
  }

  // calculate the mean
  rmse = rmse/estimations.size();

  // calculate the squared root
  rmse = rmse.array().sqrt();

  // return the result
  return rmse;
  
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
  MatrixXd Hj(3,4);
  // recover state parameters
  double px = x_state(0);
  double py = x_state(1);
  double vx = x_state(2);
  double vy = x_state(3);
  
  //std::cout<<"Px:"<<px<<"\tPy:"<<py<<"\n";
  

  // TODO: YOUR CODE HERE 
  double px_sq = (px*px);
  double py_sq= (py*py);
  
  double p = (px_sq+py_sq);
  double p_sr = sqrt(p);
  double p_cr = p*p_sr;
 

  // check division by zero
  if (fabs(p)>0.0001){
      // compute the Jacobian matrix
      Hj<< px/p_sr, py/p_sr, 0, 0,
          -py/p, px/p, 0, 0,
           py*(vx*py-vy*px)/p_cr, px*(vy*px-vx*py)/p_cr, px/p_sr, py/p_sr;
  }


  return Hj;
}
