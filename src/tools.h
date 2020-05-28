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
#include <iomanip> 
#include <ios> 

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


    rmse += residuals;
  }
  // Calculate the mean
  rmse = rmse/estimations.size();
  // Calculate the squared root
  rmse = rmse.array().sqrt();

  // RMSE should <= [.11, .11, 0.52, 0.52].
  VectorXd target(4);
  target << .11, .11, 0.52, 0.52;
  const char *true_false[2] = { "False", "True"};
  const int width = 15;

  cout<<std::setw(width)<<"rmse" 
      <<std::setw(width)<<"target" 
      <<std::setw(width)<<"meet target?"<<endl;
  for(int i=0; i<4; i++)
  {
    cout  <<std::setw(width)<<rmse[i] 
          <<std::setw(width)<<target[i] 
          <<std::setw(width)<<true_false[rmse[i] < target[i]] <<endl;
  }

}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */

  MatrixXd Hj(3,4);
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // pre-compute a set of terms to avoid repeated calculation
  float c1 = px*px+py*py;
  float c2 = sqrt(c1);
  float c3 = (c1*c2);

  // check division by zero
  if (fabs(c1) < 0.0001) {
    cout << "CalculateJacobian () - Error - Division by Zero" << endl;
    return Hj;
  }

  // compute the Jacobian matrix
  Hj << (px/c2),              (py/c2),              0,      0,
        -(py/c1),             (px/c1),              0,      0,
        py*(vx*py-vy*px)/c3,  px*(px*vy-py*vx)/c3,  px/c2,  py/c2;

  // cout << "Hj is" << Hj << endl;
  return Hj;
}


#endif  // TOOLS_H_
