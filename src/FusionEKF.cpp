#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include <math.h> 

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_laser_ << 0.0225,      0,
                   0, 0.0225;
  R_radar_ = MatrixXd(3, 3);
  R_radar_ << 0.09,      0,    0,
				 0, 0.0009,    0,
                 0,      0, 0.09;
		
  H_laser_ = MatrixXd(2, 4);
  H_laser_ << 1, 0, 0, 0,
			  0, 1, 0, 0;
  
  Hj_ = MatrixXd(3, 4);
  
  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */


  
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    x_init = VectorXd(4);
    x_init << 1, 1, 1, 1;
	
	F_init = MatrixXd(4, 4);
	F_init << 1, 0, 1, 0,
	          0, 1, 0, 1,
			  0, 0, 1, 0,
			  0, 0, 0, 1;
	
	P_init = MatrixXd(4, 4);
	P_init << 1, 0,   0,   0,
	          0, 1,   0,   0,
			  0, 0, 1000,  0,
			  0, 0,   0, 1000;
	
	Q_init = MatrixXd(4, 4);
	Q_init << 1, 0, 0, 0,
	          0, 1, 0, 0,
			  0, 0, 1, 0,
			  0, 0, 0, 1;
	
	
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
	  float ro;
      float theta;
      float ro_dot;
	  int px;
	  int py;
	  ro = measurement_pack.raw_measurements_[0]
	  theta = measurement_pack.raw_measurements_[1]
	  ro_dot = measurement_pack.raw_measurements_[2]
	  
	  px = ro * cos(theta);
	  py = ro * sin(theta);
	  
	  x_init << px, py, 0, 0;
	  
	  H_init = MatrixXd(3,4);
	  H_init << 0, 0, 0, 0,
	            0, 0, 0, 0,
				0, 0, 0, 0;
	  
	  MatrixXd R_init = R_radar_; //initialize variable separately???
	  
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
	  //set the state with the initial location and zero velocity
	  x_init << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
	  
	  H_init = MatrixXd(2,4);
	  H_init << 1, 0, 0, 0,
	            0, 1, 0, 0;
	  MatrixXd R_init = R_laser_; //initialize variable separately???
	  
    }
	
	ekf_.Init(x_init, P_init, F_init, H_init, R_init, Q_init);
	
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
    
	int noise_ax = 9;
	int noise_ay = 9;
	
   	//compute the time elapsed between the current and previous measurements
	dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000 ;	//dt - expressed in seconds
	previous_timestamp_ = measurement_pack.timestamp_;
	
    // TODO: YOUR CODE HERE
	
	//1. Modify the F matrix so that the time is integrated
	ekf_.F_(0, 2) = dt;
	ekf_.F_(1, 3) = dt;
	//2. Set the process covariance matrix Q
	ekf_.Q_ <<  dt_4/4*noise_ax,           0,       dt_3/2*noise_ax,          0,
			           0,          dt_4/4*noise_ay,        0,          dt_3/2*noise_ay,
			    dt_3/2*noise_ax,           0,       dt_2*noise_ax,            0,
			           0,          dt_3/2*noise_ay,        0,          dt_2*noise_ay;
	  

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
	Hj_ = tools.CalculateJacobian(x_) //TODO: Check calculation
	ekf_.H_ = Hj_
	ekf_.R_ = R_radar_
	
	ekf_.UpdateEKF(z);
  } else {
    // Laser updates
	ekf_.H_ = H_laser_
	ekf_.R_ = R_laser_
	ekf_.Update(z);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
