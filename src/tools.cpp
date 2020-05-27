#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::cout;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  cout<< "Calling the tools.cpp. \n";

  if(estimations.size() == 0)
  {
      cout<<"The estimations.size() CANNOT be 0. \n";
      return rmse;
  }
  if(ground_truth.size() == 0)
  {
      cout<<"The ground_truth.size() CANNOT be 0. \n";
      return rmse;
  }
  if(estimations.size() != ground_truth.size())
  {
      cout<<"The estimations and ground_truth have to be the same size. \n";
      return rmse;
  }

  // Accumulate squared residuals
  for (int i=0; i < estimations.size(); ++i) {
    VectorXd residuals = estimations[i] - ground_truth[i];
    residuals = residuals.array() * residuals.array();

    //cout << "residuals.size() is "<<residuals.size() << endl;
    //cout << residuals << endl;

    rmse += residuals;
  }
  // Calculate the mean
  rmse = rmse/estimations.size();
  // Calculate the squared root
  rmse = rmse.array().sqrt();

}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
}
