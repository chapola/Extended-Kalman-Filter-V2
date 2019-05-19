#include "kalman_filter.h"
#include <iostream>
using namespace std;

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}
const float PI2= 2*M_PI;

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
    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
    /**
     TODO:
     * update the state by using Kalman Filter equations
     */
    VectorXd z_pred = H_ * x_;
    VectorXd y = z - z_pred;
    MatrixXd Ht = H_.transpose();
    MatrixXd PHt = P_ * Ht;
    MatrixXd S = H_ *PHt + R_;
    MatrixXd Si = S.inverse();
    
    MatrixXd K = PHt * Si;
    
    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
    /**
     TODO:
     * update the state by using Extended Kalman Filter equations
     */
    double px = x_(0);
    double py = x_(1);
    double vx = x_(2);
    double vy = x_(3);
    
    double c1 = px*px+py*py;
    if (abs(c1)<0.0001) {
        cout <<"Division by 0 error"<<endl;
    }
    double rho = sqrt(c1);
    double theta = atan2(py,px);
    double rho_dot = (px*vx+py*vy)/rho;
    
    VectorXd h_x_prime(3);
    h_x_prime<<rho,theta,rho_dot;
    
    VectorXd y_ = z-h_x_prime;
    
    // Normalize the angle between -pi to pi
    while (y_(1)<-M_PI) {
        y_(1)+=PI2;
    };
    while (y_(1)>M_PI) {
        y_(1)-=PI2;
    }
    
    
    
    MatrixXd Ht = H_.transpose();
    MatrixXd S_ = H_*P_*Ht+R_;
    MatrixXd Si = S_.inverse();
    MatrixXd PHt = P_*Ht;
    MatrixXd K_ = PHt * Si;
    
    // New state
    
    x_=x_+(K_*y_);
    
    MatrixXd I = MatrixXd::Identity(4,4);
    P_ = (I - K_ * H_ ) * P_;
    
    
    
}
