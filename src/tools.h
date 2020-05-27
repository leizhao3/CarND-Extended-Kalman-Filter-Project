#ifndef TOOLS_H_
#define TOOLS_H_

#include <vector>
#include "Eigen/Dense"

class Tools {
 public:
  /**
   * Constructor.
   */
  Tools();

  /**
   * Destructor.
   */
  virtual ~Tools();

  /**
   * A helper method to calculate RMSE.
   */
  Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations,
                                const std::vector<Eigen::VectorXd> &ground_truth);

  /**
   * A helper method to calculate Jacobians.
   */
  Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state);

};


/*--------------------------start of cpp----------------------------------*/


#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::cout;
using std::endl;

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
  cout<< "Calling the tools.h. \n";

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

    cout << "residuals.size() is "<<residuals.size() << endl;
    cout << residuals << endl;

    rmse += residuals;
  }
  // Calculate the mean
  rmse = rmse/estimations.size();
  // Calculate the squared root
  rmse = rmse.array().sqrt();

  cout<<"rmse is "<<rmse;

}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
}




#endif  // TOOLS_H_
