#define _USE_MATH_DEFINES
#include "kalman_filter.h"
#include "tools.h"
#include <iostream>
#include <math.h>

using namespace std;


using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize
// VectorXd or MatrixXd objects with zeros upon creation.

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
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_; //TODO: add u or not?
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */

  VectorXd y = z - H_ * x_;
  cout << "z:" << z.rows() << " " <<  z.cols() << endl;
  cout << "H_:" << H_.rows() << " " <<  H_.cols() << endl;
  cout << "x_:" << x_.rows() << " " <<  x_.cols() << endl;
  cout << "y:" << y.rows() << " " <<  y.cols() << endl;

  MatrixXd Ht = H_.transpose(); //4,2
  MatrixXd S = H_ * P_ * Ht + R_; // 2,4  4,4 4,2
  cout << "S:" << S.rows() << " " <<  S.cols() << endl;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;
  cout << "K:" << K.rows() << " " <<  K.cols() << endl;

  x_ = x_ + (K * y);
  cout << "x_2:" << x_.rows() << " " <<  x_.cols() << endl;
  MatrixXd I = MatrixXd::Identity(4, 4);
  P_ = (I -  K * H_) * P_;
  cout << "P_:" << P_.rows() << " " <<  P_.cols() << endl;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  //TODO: normalize the angle?
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);
  float rho, phi, rho_dot;
  if(px == 0 and py == 0){
	  rho = 0;
	  phi = 0;
	  rho_dot = 0;
  } else if (px==0){
	  rho = py;
      phi = M_PI/2;
      rho_dot = vy;
  } else {
	  rho = sqrt(px*px + py*py);
	  phi = atan2(py, px);
	  rho_dot = (px*vx+py*vy)/rho;
  }

  VectorXd hx(3);
  hx << rho, phi, rho_dot;

  Tools tools;
  cout << "z:" << z.rows() << " " <<  z.cols() << endl;
  cout << "hx:" << hx.rows() << " " <<  hx.cols() << endl;
  VectorXd y = z - hx;
  // Normalizing Angles, see Tips and Tricks from project document
  if(y(1) > M_PI){
	  y(1) = 2*M_PI - y(1);
  }else if(y(1) < -M_PI){
	  y(1) = -(y(1) + 2*M_PI);
  }
  cout << "y:" << y.rows() << " " <<  y.cols() << endl;

  H_ = tools.CalculateJacobian(x_);
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  cout << "S:" << S.rows() << " " <<  S.cols() << endl;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;
  cout << "K:" << K.rows() << " " <<  K.cols() << endl;

  x_ = x_ + (K * y);
  cout << "x_2:" << x_.rows() << " " <<  x_.cols() << endl;
  MatrixXd I = MatrixXd::Identity(4, 4);
  P_ = (I -  K * H_) * P_;
  cout << "P_:" << P_.rows() << " " <<  P_.cols() << endl;

}
