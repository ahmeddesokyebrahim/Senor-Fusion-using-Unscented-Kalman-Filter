#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
//#include <math.h>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.25;
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  is_initialized_ = false;

  time_us_ = 0;

  P_ << 1 , 0 , 0 , 0 , 0 ,
		0 , 1 , 0 , 0 , 0 ,
		0 , 0 , 1 , 0 , 0 ,
		0 , 0 , 0 , 1 , 0 ,
		0 , 0 , 0 , 0 , 1 ;

  n_x_ = 5;

  n_aug_ = 7;

  lambda_ = 3 - n_aug_;

  weights_= VectorXd(2*n_aug_+1);
  // set weights
  double weight_0 = lambda_/(lambda_+n_aug_);
  weights_(0) = weight_0;
  for (int i=1; i<2*n_aug_+1; i++) {  //2n+1 weights
    double weight = 0.5/(n_aug_+lambda_);
    weights_(i) = weight;
  }

  Xsig_pred_ = MatrixXd(n_x_, 2*n_aug_+1);
  Xsig_pred_.fill(0.0);

}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */

	/*****************************************************************************
	 *  Initialization
	 ****************************************************************************/
	if(!is_initialized_){
		if(meas_package.sensor_type_ == MeasurementPackage::RADAR){
			float rho = meas_package.raw_measurements_[0];
			float theta = meas_package.raw_measurements_[1];
			float rho_dot = meas_package.raw_measurements_[2];
			float px = rho * cos(theta);
			float py = rho * sin(theta);
			float v = 0.0;
			float yaw_angle = 0.0;
			float yaw_rate = 0.0;

			x_ << px , py , v , yaw_angle , yaw_rate;
			//cout << "Radar Init Done" << endl;
		}
		else if(meas_package.sensor_type_ == MeasurementPackage::LASER){
			float px = meas_package.raw_measurements_[0];
			float py = meas_package.raw_measurements_[1];
			float v = 0.0;
			float yaw_angle = 0.0;
			float yaw_rate = 0.0;

			x_ << px , py , v , yaw_angle , yaw_rate;
			//cout << "Laser Init Done" << endl;
		}

		time_us_ = meas_package.timestamp_;

		is_initialized_ = true;

		//cout << "Initialization Done" << endl;

		return;
	}

	double delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0;
	time_us_ = meas_package.timestamp_;

	/*****************************************************************************
	 *  Prediction
	 ****************************************************************************/

	Prediction(delta_t);

	/*****************************************************************************
	 *  Update
	 ****************************************************************************/

	if(meas_package.sensor_type_ == MeasurementPackage::RADAR){
		//cout << "I am in Radar Update" << endl;
		UpdateRadar(meas_package);
	}
	else{
		//cout << "I am in Lidar Update" << endl;
		UpdateLidar(meas_package);
	}

	/*Print the output*/
	cout << "x_ = " << x_ << endl;
	cout << "P_ = " << P_ << endl;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
	//create augmented mean vector
	VectorXd x_aug = VectorXd(7);

	//create augmented state covariance
	MatrixXd P_aug = MatrixXd(7, 7);

	//create sigma point matrix
	MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

	//cout << "I am in Prediction Function" << endl;

	/*****************************************************************************
	 *  Generate Sigma Points
	 ****************************************************************************/

	//create augmented mean state
	x_aug.head(5) = x_;
	x_aug(5) = 0;
	x_aug(6) = 0;

	 //create augmented covariance matrix
	P_aug.fill(0.0);
	P_aug.topLeftCorner(5, 5) = P_;
	P_aug(5, 5) = std_a_ * std_a_;
	P_aug(6, 6) = std_yawdd_ * std_yawdd_;

	//create square root matrix
	MatrixXd L = P_aug.llt().matrixL();

	//create augmented sigma points
	Xsig_aug.col(0) = x_aug;
	for (int i = 0; i < n_aug_; i++) {
		Xsig_aug.col(i + 1) = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
		Xsig_aug.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
	}

	//cout << "Generate Sigma Points is Done" << endl;
	/*****************************************************************************
	 *  Predict Sigma Points
	 ****************************************************************************/

	//predict sigma points
	for (int i = 0; i < 2*n_aug_+1; i++) {
		//extract values for better readability
		double p_x = Xsig_aug(0, i);
		double p_y = Xsig_aug(1, i);
		double v = Xsig_aug(2, i);
		double yaw = Xsig_aug(3, i);
		double yawd = Xsig_aug(4, i);
		double nu_a = Xsig_aug(5, i);
		double nu_yawdd = Xsig_aug(6, i);

		//predicted state values
		double px_p, py_p;

		//avoid division by zero
		if (fabs(yawd) > 0.001) {
			px_p = p_x + v / yawd * (sin(yaw + yawd * delta_t) - sin(yaw));
			py_p = p_y + v / yawd * (cos(yaw) - cos(yaw + yawd * delta_t));
		} else {
			px_p = p_x + v * delta_t * cos(yaw);
			py_p = p_y + v * delta_t * sin(yaw);
		}

		double v_p = v;
		double yaw_p = yaw + yawd * delta_t;
		double yawd_p = yawd;

		//add noise
		px_p = px_p + 0.5 * nu_a * delta_t * delta_t * cos(yaw);
		py_p = py_p + 0.5 * nu_a * delta_t * delta_t * sin(yaw);
		v_p = v_p + nu_a * delta_t;

		yaw_p = yaw_p + 0.5 * nu_yawdd * delta_t * delta_t;
		yawd_p = yawd_p + nu_yawdd * delta_t;

		//write predicted sigma point into right column
		Xsig_pred_(0, i) = px_p;
		Xsig_pred_(1, i) = py_p;
		Xsig_pred_(2, i) = v_p;
		Xsig_pred_(3, i) = yaw_p;
		Xsig_pred_(4, i) = yawd_p;
	}
	//cout << "Predict Sigma Points is Done" << endl;
	/*****************************************************************************
	 *  Predict Mean and Covariance
	 ****************************************************************************/

	//predicted state mean
	x_.fill(0.0);
	for (int i = 0; i < 2*n_aug_+ 1; i++) {  //iterate over sigma points
		x_ = x_ + weights_(i) * Xsig_pred_.col(i);
	}
	//cout << "Predict State Mean" << endl;

	//predicted state covariance matrix
	P_.fill(0.0);
	for (int i = 0; i < 2*n_aug_+1; i++) {  //iterate over sigma points

		// state difference
		VectorXd x_diff = Xsig_pred_.col(i) - x_;
		//angle normalization
		while (x_diff(3) > M_PI){
			x_diff(3) = x_diff(3) - (2.0 * M_PI);
			//cout << "First Norm" << endl;
			//cout << "x_diff_3 : " << x_diff(3) << endl;
		}
		while (x_diff(3) < -M_PI){
			x_diff(3)+=2.*M_PI;
			//cout << "Second Norm" << endl;
			//cout << "x_diff_3 : " << x_diff(3) << endl;
		}

		P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
	}
	//cout << "Predict Mean and Covariance is Done" << endl;
}


/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
	/*****************************************************************************
	 *  Predict Measurements
	 ****************************************************************************/

	//set measurement dimension, laser can measure px and py
	int n_z = 2;

	//create matrix for sigma points in measurement space
	MatrixXd Zsig = MatrixXd(n_z, 2*n_aug_+1);

	//create matrix for cross correlation Tc
	MatrixXd Tc = MatrixXd(n_x_, n_z);
	Tc.fill(0.0);

	//transform sigma points into measurement space
	for (int i = 0; i < 2*n_aug_+1; i++) {  //2n+1 simga points

		// extract values for better readibility
		double p_x = Xsig_pred_(0, i);
		double p_y = Xsig_pred_(1, i);

		// measurement model
		Zsig(0, i) = p_x;      //px
		Zsig(1, i) = p_y;      //py
	}
	//cout << "Lidar sigma points transformation is done" << endl;

	//mean predicted measurement
	VectorXd z_pred = VectorXd(n_z);
	z_pred.fill(0.0);
	for (int i = 0; i < 2*n_aug_+1; i++) {
		z_pred = z_pred + weights_(i) * Zsig.col(i);
	}
	//cout << "Lidar meas predict is done" << endl;

	//innovation covariance matrix S
	MatrixXd S = MatrixXd(n_z, n_z);
	S.fill(0.0);
	for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
		//residual
		VectorXd z_diff = Zsig.col(i) - z_pred;

		S = S + weights_(i) * z_diff * z_diff.transpose();
	}
	//cout << "Lidar innovation covariance matrix s is done" << endl;

	//add measurement noise covariance matrix
	MatrixXd R = MatrixXd(n_z, n_z);
	R << std_laspx_ * std_laspx_, 0,
		 0, std_laspy_ * std_laspy_;

	S = S + R;


	/*****************************************************************************
	 *  Update State
	 ****************************************************************************/

	//create example vector for incoming radar measurement
	VectorXd z = VectorXd(n_z);
	z << meas_package.raw_measurements_[0],
		 meas_package.raw_measurements_[1];

	for (int i = 0; i < 2*n_aug_+1; i++) {  //2n+1 simga points

		//residual
		VectorXd z_diff = Zsig.col(i) - z_pred;

		// state difference
		VectorXd x_diff = Xsig_pred_.col(i) - x_;

		Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
	}
	//cout << "Lidar TC is done" << endl;

	//Kalman gain K;
	MatrixXd K = Tc * S.inverse();

	//residual
	VectorXd z_diff = z - z_pred;

	//update state mean and covariance matrix
	x_ = x_ + K * z_diff;
	P_ = P_ - K * S * K.transpose();
	//cout << "Lidar Update is done" << endl;
	float lidar_nis = z_diff.transpose() * S.inverse() * z_diff;
	cout << "Lidar NIS = " << lidar_nis << endl;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */

	/*****************************************************************************
	 *  Predict Measurements
	 ****************************************************************************/

	//set measurement dimension, radar can measure r, phi, and r_dot
	int n_z = 3;

	//create matrix for sigma points in measurement space
	MatrixXd Zsig = MatrixXd(n_z, 2*n_aug_+1);

	//create matrix for cross correlation Tc
	MatrixXd Tc = MatrixXd(n_x_, n_z);
	Tc.fill(0.0);

	//transform sigma points into measurement space
	for (int i = 0; i < 2*n_aug_+1; i++) {  //2n+1 simga points

		// extract values for better readibility
		double p_x = Xsig_pred_(0, i);
		double p_y = Xsig_pred_(1, i);
		double v = Xsig_pred_(2, i);
		double yaw = Xsig_pred_(3, i);

		double v1 = cos(yaw) * v;
		double v2 = sin(yaw) * v;

		// measurement model
		Zsig(0, i) = sqrt(p_x * p_x + p_y * p_y);                        //r
		Zsig(1, i) = atan2(p_y, p_x);                                 //phi
		Zsig(2, i) = (p_x * v1 + p_y * v2) / sqrt(p_x * p_x + p_y * p_y); //r_dot
	}
	//cout << "Radar Sigma Points transform is done"<< endl;

	//mean predicted measurement
	VectorXd z_pred = VectorXd(n_z);
	z_pred.fill(0.0);
	for (int i = 0; i < 2*n_aug_+1; i++) {
		z_pred = z_pred + weights_(i) * Zsig.col(i);
	}
	//cout << "Radar Mean Predict is Done"<< endl;

	//innovation covariance matrix S
	MatrixXd S = MatrixXd(n_z, n_z);
	S.fill(0.0);
	for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
		//residual
		VectorXd z_diff = Zsig.col(i) - z_pred;

		//angle normalization
		while (z_diff(1) > M_PI)
			z_diff(1) -= 2. * M_PI;
		while (z_diff(1) < -M_PI)
			z_diff(1) += 2. * M_PI;

		S = S + weights_(i) * z_diff * z_diff.transpose();
	}
	//cout << "Radar Innovation covariance matrix s is done"<< endl;

	//add measurement noise covariance matrix
	MatrixXd R = MatrixXd(n_z, n_z);
	R << std_radr_ * std_radr_, 0, 0,
		 0, std_radphi_ * std_radphi_, 0,
		 0, 0, std_radrd_ * std_radrd_;
	S = S + R;

	/*****************************************************************************
	 *  Update State
	 ****************************************************************************/

	//create example vector for incoming radar measurement
	VectorXd z = VectorXd(n_z);
	z << meas_package.raw_measurements_[0],
		 meas_package.raw_measurements_[1],
		 meas_package.raw_measurements_[2];

	for (int i = 0; i < 2*n_aug_+1; i++) {  //2n+1 simga points

		//residual
		VectorXd z_diff = Zsig.col(i) - z_pred;
		//angle normalization
		while (z_diff(1) > M_PI)
			z_diff(1) -= 2. * M_PI;
		while (z_diff(1) < -M_PI)
			z_diff(1) += 2. * M_PI;

		// state difference
		VectorXd x_diff = Xsig_pred_.col(i) - x_;
		//angle normalization
		while (x_diff(3) > M_PI)
			x_diff(3) -= 2. * M_PI;
		while (x_diff(3) < -M_PI)
			x_diff(3) += 2. * M_PI;

		Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
	}
	//cout << "Radar TC is done "<< endl;

	//Kalman gain K;
	MatrixXd K = Tc * S.inverse();

	//residual
	VectorXd z_diff = z - z_pred;

	//angle normalization
	while (z_diff(1) > M_PI)
		z_diff(1) -= 2. * M_PI;
	while (z_diff(1) < -M_PI)
		z_diff(1) += 2. * M_PI;

	//update state mean and covariance matrix
	x_ = x_ + K * z_diff;
	P_ = P_ - K * S * K.transpose();
	//cout << "Radar Update is Done"<< endl;

	float radar_nis = z_diff.transpose() * S.inverse() * z_diff;
	cout << "Radar NIS = " << radar_nis << endl;
}
