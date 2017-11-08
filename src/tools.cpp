#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  //check the validity of the inputs
  int esti_size = estimations.size();
  if((esti_size == 0) || (esti_size != ground_truth.size())){
    return rmse;
  }

  for(int i=0; i < esti_size; ++i){
    VectorXd diff = estimations[i] - ground_truth[i];
    diff = diff.array() * diff.array();
    rmse += diff;
  }

  rmse = rmse/esti_size;
  rmse = rmse.array().sqrt();

  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  MatrixXd Hj(3, 4);

  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  float p_xy = px*px + py*py;
  float p_xy_sq = sqrt(p_xy);
  float p_xy_3_2 = p_xy * p_xy_sq;

  //check division by zero
  if(abs(p_xy) <= 0.0001){
    return Hj;
  }

  //compute the Jacobian matrix
  Hj << px/p_xy_sq, py/p_xy_sq, 0, 0,
        (-py)/p_xy, px/p_xy, 0, 0,
        py*(vx*py-vy*px)/p_xy_3_2, px*(vy*px-vx*py)/p_xy_3_2,
        px/p_xy_sq, py/p_xy_sq;
  return Hj;
}
