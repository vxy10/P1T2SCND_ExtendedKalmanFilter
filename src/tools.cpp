#include <iostream>
#include "tools.h"

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

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	if(estimations.size() != ground_truth.size()
			|| estimations.size() == 0){
		cout << "Invalid estimation or ground_truth data" << endl;
		return rmse;
	}

	//accumulate squared residuals
	for(unsigned int i=0; i < estimations.size(); i++){

		VectorXd residual = estimations[i] - ground_truth[i];

        //cout << residual << endl;
		//coefficient-wise multiplication
		residual = residual.array()*residual.array();
		rmse += residual;
	}

	//calculate the mean
	rmse = rmse/estimations.size();

	//calculate the squared root
	rmse = rmse.array().sqrt();

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

	if ((pow(px,2)+pow(py,2))==0){
	    cout<<"Divide by 0";
	    return Hj;
	}else{

	    Hj<<px/pow(pow(px,2)+pow(py,2),.5),
	    py/pow(pow(px,2)+pow(py,2),.5),
	    0, 0,
            -py/(pow(px,2) + pow(py,2)),
            px/(pow(px,2) + pow(py,2)),
            0, 0,
            -pow(px,2)*vx/pow(pow(px,2)+pow(py,2),1.5)-px*py*vy/pow(pow(px,2)+pow(py,2),1.5)+vx/sqrt(pow(px,2)+pow(py,2)),
            -px*py*vx/pow(pow(px,2)+pow(py,2),1.5)-pow(py,2)*vy/pow(pow(px,2)+pow(py,2),1.5)+vy/sqrt(pow(px,2)+pow(py,2)),
            px/pow(pow(px,2)+pow(py,2),.5),
            py/pow(pow(px,2)+pow(py,2),.5);


	    //compute the Jacobian matrix
	    return Hj;
	}




}
