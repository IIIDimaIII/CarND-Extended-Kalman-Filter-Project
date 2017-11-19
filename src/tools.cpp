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
  if(estimations.size()<0){
	rmse << -1,-1,-1,-1;
    return rmse;};
  if(estimations.size()!=ground_truth.size()){
	rmse << -1,-1,-1,-1;
    return rmse;};
  
  //accumulate squared residuals
  for(int i=0; i < estimations.size(); ++i){
	for(int j=0; j<4;++j){
	  rmse[j] += pow(estimations[i][j]-ground_truth[i][j],2);
    }
  }

  //calculate the mean
  for(int j=0; j<4;++j){
	rmse[j] /= 3;
  }

  //calculate the squared root
  for(int j=0; j<4;++j){
	rmse[j] = pow(rmse[j],0.5);
  }
  
  //return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  MatrixXd Hj(3,4);
  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  //TODO: YOUR CODE HERE
  //check division by zero
  float d = pow(pow(px,2)+ pow(py,2), 0.5);
    
  //compute the Jacobian matrix
  if(d>0.0001){
    Hj <<  			   px/d, 						py/d, 			    0,    0,
				  -py/pow(d,2), 				px/pow(d,2), 			0,    0,
		  py*(vx*py -vy*px)/pow(d,3/2), px*(vy*px-vx*py)/pow(d,3/2),  px/d, py/d;
  };
  return Hj;
}
