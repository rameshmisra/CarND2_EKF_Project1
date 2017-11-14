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
	rmse << 0,0,0,0;

	if(estimations.size() != ground_truth.size()
			|| estimations.size() == 0){
		cout << "Invalid estimation or ground_truth data" << endl;
		return rmse;
	}

	VectorXd resid;
	for(int i=0; i < estimations.size(); ++i){
        
        resid = (estimations[i] - ground_truth[i]);
        resid = resid.array()*resid.array();
		rmse += resid;
	}

	rmse = rmse/estimations.size();
	rmse = rmse.array().sqrt();

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
	float rho = sqrt(px*px  + py*py);
	float rho3 = rho*rho*rho;
	if (fabs(rho) < 0.001){
	    cout<<"Division by zero error in CalculateJacobian()";
	    return Hj;
	}
	else {
	    Hj << px/rho, py/rho, 0, 0,
	     -py/(rho*rho), px/(rho*rho), 0, 0,
	     py*(vx*py-vy*px)/rho3, px*(vy*px-vx*py)/rho3, px/rho, py/rho;
	}
	
	     

	//check division by zero
	
	//compute the Jacobian matrix

	return Hj;
}
