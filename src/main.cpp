#include <iostream>
#include <iostream>
#include <random>
#include <chrono>
#include <fstream>
#include <vector>
#include <cmath>

#include "SystemModel.hpp"
#include "OdomMeasurementModel.hpp"
#include "ImuMeasuremenntModel.hpp"
#include "kalman/ExtendedKalmanFilter.hpp"
#include "kalman/Types.hpp"
#include "rapidcsv.h"
#include "ThreeWheel.hpp"
#include "TFMiniMeasurementModek.hpp"
//#include "TwoDistanceSensorsMeasurementModel.hpp"
#include <Eigen/Dense>

#include "filter.h"
#include "three_wheels_to_twist.hpp"

#define MID_RADIUS 0.235

// #define RIGHT_RADIUS 0.2425
// #define LEFT_RADIUS 0.2425

#define RIGHT_RADIUS 0.32
#define LEFT_RADIUS 0.165

#define WHEEL_D 0.0574
//#define WHEEL_D 0.0474





typedef double T;

typedef Robot::State<T> State;
typedef Robot::Twist<T> Twist;
typedef Robot::SystemModel<T> SystemModel;

typedef Robot::OdomMeasurement<T> OdomMeasurement;
typedef Robot::ImuMeasurement<T> ImuMeasurement;
typedef Robot::WheelMeasurement<T> WheelMeasurement;
typedef Robot::TFMiniMeasurement<T> TFMiniMeasurement;
//typedef Robot::TwoSensorsMeasurement<T> TwoSensorsMeasurement;


typedef Robot::OdomMeasurementModel<T> OdomMeasurementModel;
typedef Robot::ImuMeasurementModel<T> ImuMeasurementModel;
typedef Robot::WheelMeasurementModel<T> WheelMeasurementModel;
typedef Robot::TFMiniMeasurementModel<T> TFMiniMeasurementModel;
//typedef Robot::TwoSensorsMeasurementModel<T> TwoSensorsMeasurementModel;


template <typename T>
T degreesToRadians(T degrees) {
  return degrees * static_cast<T>(M_PI) / static_cast<T>(180);
}



// Function to rotate a vector using roll (X-axis) and pitch (Y-axis)
Eigen::Vector3d rotateVector(const Eigen::Vector3d &v, double roll, double pitch) {
    // Create roll rotation matrix (rotation about X-axis)
    Eigen::Matrix3d R_roll = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()).toRotationMatrix();
    
    // Create pitch rotation matrix (rotation about Y-axis)
    Eigen::Matrix3d R_pitch = Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()).toRotationMatrix();
    
    // Combine the rotations: roll first, then pitch
    Eigen::Matrix3d R = R_pitch * R_roll;
    
    // Apply the rotation to the vector
    return R * v;
}


double get_cov(double val) {
  // if (val != 70)
  //   return (1700 * 0.4 /(val - 70));
  // else return 100;

  if (val < 700) return 70;
  //  else return 0.09;
  else return 0.09;
}

int main() {
  // Open a file To store the csv file

  std::ofstream fs("../data/data.csv", std::ios::out);

  std::cout << "Hello Kalman Filter" << std::endl;

  State state;
  state.setZero();

  state.x() = 1;
  state.y() = 1;

  Twist twist;


  OdomMeasurementModel odom_model;
  ImuMeasurementModel imu_model;



  //                                x   y           a  x    y            a   x          y    a        rad 
  WheelMeasurementModel wheel_model(0, LEFT_RADIUS, 0, 0, -RIGHT_RADIUS, 0, -MID_RADIUS, 0 , PI/2 , WHEEL_D/2);
  SystemModel sys;

  TFMiniMeasurementModel mini_x_plus(-PI/2, 0.265, 0, 0.295, PI/2, 0.273, 8, 15);

  TFMiniMeasurementModel mini_y_plus(0, 0.295, PI/2, 0.273, PI, 0.275, 8, 15);

  TFMiniMeasurementModel mini_x_minus(PI/2, 0.273, PI, 0.275, -PI/2, 0.265, 8, 15);

  TFMiniMeasurementModel mini_y_minus(PI, 0.275, -PI/2, 0.265, 0, 0.295, 8, 15);

  Kalman::Covariance<OdomMeasurement> odom_cov;
  Kalman::Covariance<ImuMeasurement> imu_cov;
  Kalman::Covariance<WheelMeasurement> wheel_cov;
  Kalman::Covariance<State> state_cov;

  Kalman::Covariance<TFMiniMeasurement> x_plus_cov;
  Kalman::Covariance<TFMiniMeasurement> x_minus_cov;
  Kalman::Covariance<TFMiniMeasurement> y_plus_cov;
  Kalman::Covariance<TFMiniMeasurement> y_minus_cov;

  x_plus_cov.setIdentity();
  x_plus_cov /= 50;

  y_plus_cov.setIdentity();
  y_plus_cov /= 50;

  x_minus_cov.setIdentity();
  x_minus_cov /= 50;
 
  y_minus_cov.setIdentity();
  y_minus_cov /= 50;


  mini_x_plus.setCovariance(x_plus_cov);
  mini_y_plus.setCovariance(y_plus_cov);
  mini_x_minus.setCovariance(x_minus_cov);
  mini_y_minus.setCovariance(y_minus_cov);
 
  // Kalman::Covariance<TwoSensorsMeasurement> cov_right;
  // Kalman::Covariance<TwoSensorsMeasurement> cov_left;

  state_cov.setIdentity();
  state_cov /= 10;

  state_cov(State::X, State::X) = 1;
  state_cov(State::Y, State::Y) = 1;
  state_cov(State::VX, State::VX) = 0.3;
  state_cov(State::VY, State::VY) = 0.3;
  state_cov(State::OMEGA, State::OMEGA) = 0.3;
  state_cov(State::THETA, State::THETA) = 2;
  state_cov(State::AX, State::AX) = 1;
  state_cov(State::AY, State::AY) = 1;

  wheel_cov(WheelMeasurement::OMEGA_L, WheelMeasurement::OMEGA_L) = 1;
  wheel_cov(WheelMeasurement::OMEGA_R, WheelMeasurement::OMEGA_R) = 1;
  wheel_cov(WheelMeasurement::OMEGA_M, WheelMeasurement::OMEGA_M) = 1;

  imu_cov(ImuMeasurement::AX, ImuMeasurement::AX) = 1;
  imu_cov(ImuMeasurement::AY, ImuMeasurement::AY) = 1;
  imu_cov(ImuMeasurement::YAW, ImuMeasurement::YAW) = 0.001;


  // odom_cov(OdomMeasurement::X, OdomMeasurement::X) = 100;
  // odom_cov(OdomMeasurement::Y, OdomMeasurement::Y) = 100;
  // odom_cov(OdomMeasurement::THETA, OdomMeasurement::THETA) = 0.1;

  odom_cov(OdomMeasurement::VX, OdomMeasurement::VX) = 0.000625;
  odom_cov(OdomMeasurement::VY, OdomMeasurement::VY) = 0.003;

  // odom_cov(OdomMeasurement::VX, OdomMeasurement::VX) = 0.000625;
  // odom_cov(OdomMeasurement::VY, OdomMeasurement::VY) = 0.003;
  odom_cov(OdomMeasurement::OMEGA, OdomMeasurement::OMEGA) = 0.1;


  odom_model.setCovariance(odom_cov);
  imu_model.setCovariance(imu_cov);
  wheel_model.setCovariance(wheel_cov);

  // mini.setCovariance(tf_cov);

  /// Set Covariance Of State Model
  sys.setCovariance(state_cov);
 
  Kalman::ExtendedKalmanFilter<State> predictor;
  Kalman::ExtendedKalmanFilter<State> ekf;

  predictor.init(state);
  ekf.init(state);



  // twist.rvx() = 1;
  // twist.rvy() = 2;
  // twist.romega() = 3;

  // Get The CSV File

  rapidcsv::Document csv("../real_life_dta/10-rot.csv");

  std::vector rx = csv.GetColumn<float>(1);
  std::vector ry = csv.GetColumn<float>(2);
  std::vector rw = csv.GetColumn<float>(3);
  std::vector t = csv.GetColumn<float>(0);

  std::vector ox = csv.GetColumn<float>(4);
  std::vector oy = csv.GetColumn<float>(5);
  std::vector oth = csv.GetColumn<float>(6);
  // std::vector ovx = csv.GetColumn<float>(7);
  // std::vector ovy = csv.GetColumn<float>(8);
  // std::vector ow = csv.GetColumn<float>(9);


  std::vector o_l = csv.GetColumn<float>(4);
  std::vector o_r = csv.GetColumn<float>(5);
  std::vector o_m = csv.GetColumn<float>(6);



  std::vector ir  = csv.GetColumn<float>(10);
  std::vector ip  = csv.GetColumn<float>(11);
  std::vector iy  = csv.GetColumn<float>(12);
  std::vector iax = csv.GetColumn<float>(13);
  std::vector iay = csv.GetColumn<float>(14);
  std::vector iaz = csv.GetColumn<float>(15);

  std::vector dm = csv.GetColumn<float>(16);
  std::vector dr = csv.GetColumn<float>(17);
  std::vector dl = csv.GetColumn<float>(18);

  std::vector sm = csv.GetColumn<float>(19);
  std::vector sr = csv.GetColumn<float>(20);
  std::vector sl = csv.GetColumn<float>(21);

  std::vector sd1 = csv.GetColumn<float>(22);
  std::vector sd2 = csv.GetColumn<float>(23);
  std::vector sd3 = csv.GetColumn<float>(24);
  std::vector sd4 = csv.GetColumn<float>(25);

  std::vector sw1 = csv.GetColumn<int>(26);
  std::vector sw2 = csv.GetColumn<int>(27);
  std::vector sw3 = csv.GetColumn<int>(28);
  std::vector sw4 = csv.GetColumn<int>(29);

  
  std::vector s_up = csv.GetColumn<int>(30);

  char c;

  // Set Up 1st order and Second order low pass filters
  // As and Bs are calculated from GNU octave
  // Use GNU octave signal package and butter function
  double omega_bs[2] = {0.086364, 0.086364};
  double omega_as[2] = {1.0000, -0.8273};
  filter<1> omega_l_lpf(omega_bs, omega_as);
  filter<1> omega_r_lpf(omega_bs, omega_as);
  filter<1> omega_m_lpf(omega_bs, omega_as);
  filter<1> yaw_lpf(omega_bs, omega_as);

  double accel_bs[3] = {9.4469e-04, 1.8894e-03, 9.4469e-04};
  double accel_as[3] = {1.0000, -1.9112, 0.9150};
  filter<2> accel_x_lpf(accel_bs, accel_as);
  filter<2> accel_y_lpf(accel_bs, accel_as);
  filter<2> accel_z_lpf(accel_bs, accel_as);

  // Create controller instance
  Controller controller_model{
      -8.9/100,  -17.6/100, 0,        // Wheel 1 position and angle
      13.81/100, 0.61/100,  M_PI / 2, // Wheel 2 position and angle
      7.3/100,   16.4/100,  0,        // Wheel 3 position and angle
      WHEEL_D / 2                         // Wheel radius
  };

  double time = 0.01;

  for (int i = 00; i < rx.size(); i++) {

    time = t[i] - t[i - 1];

    // state = sys.f(state, twist, time);

    //    auto x_ekf = ekf.predict(sys, twist, time);
    auto x_pred = predictor.predict(sys, twist, time);

    double omega_r = omega_r_lpf.update(o_r[i]);
    double omega_l = omega_l_lpf.update(o_l[i]);
    double omega_m = omega_m_lpf.update(o_m[i]);

    auto calculated = controller_model.get_output(omega_l, omega_r, omega_m);
    double omega = calculated.w;
    double vx = calculated.vx;
    double vy = calculated.vy;

    OdomMeasurement mea;

    twist.rvx() = vx;
    twist.rvy() = vy;
    twist.romega() = omega;
    auto x_ekf = ekf.predict(sys, twist, time);

    // Imu Measurement
    // We can measure the orientation every 5th step
    ImuMeasurement imu;
    // Measurement is affected by noise as well

    double roll = ir[i];
    double pitch = ip[i];
    double yaw = iy[i];
    double ax = iax[i];
    double ay = iay[i];
    double az = iaz[i];

    Eigen::Vector3d accel = {ax, ay, az};

    Eigen::Vector3d rot_accel = rotateVector(accel, roll, pitch);

    // imu.yaw() = iy[i];
    // imu.ax() = accel_x_lpf.update(iax[i]);
    // imu.ay() = accel_y_lpf.update(iay[i]);

    imu.yaw() = yaw;
    imu.ax() = accel_x_lpf.update(rot_accel(0));
    imu.ay() = accel_y_lpf.update(rot_accel(1));

    // std::cout << "The accels are  " << rot_accel(0) << " " << rot_accel(1)
    // << " "  << rot_accel(2) << std::endl;
    std::cout << "roll: " << roll << std::endl;
    std::cout << "pitch : " << pitch << std::endl;
    std::cout << "az : " << az << std::endl;
    std::cout << "accel: " << accel << std::endl;
    std::cout << "rot_accel: " << rot_accel << std::endl;
    std::cout << "imu: " << imu << std::endl;

    // Update EK// F

    x_ekf = ekf.update(imu_model, imu, time);

    if (s_up[i]) {

      {
        TFMiniMeasurement mea_x_plus;
        TFMiniMeasurement mea_x_minus;
        TFMiniMeasurement mea_y_plus;
        TFMiniMeasurement mea_y_minus;


	mea_x_plus.d1() = sd4[i]/100;
	mea_x_plus.d2() = sd1[i]/100;
	mea_x_plus.d3() = sd2[i]/100;

	mea_y_plus.d1() = sd1[i]/100;
	mea_y_plus.d2() = sd2[i]/100;
	mea_y_plus.d3() = sd3[i]/100;

	mea_x_minus.d1() = sd2[i]/100;
	mea_x_minus.d2() = sd3[i]/100;
	mea_x_minus.d3() = sd4[i]/100;

        mea_y_minus.d1() = sd3[i]/100;
	mea_y_minus.d2() = sd4[i]/100;
	mea_y_minus.d3() = sd1[i]/100;

	
	if (sw4[i] && sw1[i] && sw2[i]) x_ekf = ekf.update(mini_x_plus, mea_x_plus, true, 1);

	if (sw1[i] && sw2[i] && sw3[i]) x_ekf = ekf.update(mini_y_plus, mea_y_plus, true, 1);

	if (sw2[i] && sw3[i] && sw4[i]) x_ekf = ekf.update(mini_x_minus, mea_x_minus, true, 1);

	if (sw3[i] && sw4[i] && sw1[i]) x_ekf = ekf.update(mini_y_minus, mea_y_minus, true, 1);

	// if (sw4[i] && sw1[i] && sw2[i]) x_ekf = ekf.update(mini_x_plus, mea_x_plus);

	// if (sw1[i] && sw2[i] && sw3[i]) x_ekf = ekf.update(mini_y_plus, mea_y_plus);

	// if (sw2[i] && sw3[i] && sw4[i]) x_ekf = ekf.update(mini_x_minus, mea_x_minus);

	// if (sw3[i] && sw4[i] && sw1[i]) x_ekf = ekf.update(mini_y_minus, mea_y_minus);


      }
    }

    fs << i << "," << x_ekf.vx() << "," << x_ekf.vy() << "," << state.theta()
       << "," << x_ekf.x() << "," << x_ekf.y() << "," << x_ekf.theta() << ","
       << x_pred.x() << "," << x_pred.y() << "," << x_pred.theta() << ","
       << ox[i] << "," << oy[i] << "," << oth[i] << "," << iy[i] << ","
       << 2 * sqrt(ekf.getCovariance()(State::X, State::X)) << ","
       << 2 * sqrt(ekf.getCovariance()(State::X, State::X)) << std::endl;

    twist.romega() = rw[i];
    twist.rvx() = rx[i];
    twist.rvy() = ry[i];
  }

  // check if this works properly

  return 0;
}

// {

//   TwoSensorsMeasurement right;
//   TwoSensorsMeasurement left;

//   // min.d1() = dm[i]/100;
//   // min.d3() = dl[i]/100;
//   // min.d2() = dr[i]/100;

//   right.d1() = dm[i]/100;
//   right.d2() = dr[i]/100;

//   left.d1() = dm[i]/100;
//   left.d2() = dl[i]/100;

//   cov_right(TwoSensorsMeasurement::D1, TwoSensorsMeasurement::D1) = 0.09;
//   cov_right(TwoSensorsMeasurement::D2, TwoSensorsMeasurement::D2) = 0.09;

//   cov_left(TwoSensorsMeasurement::D1, TwoSensorsMeasurement::D1) = 0.09;
//   cov_left(TwoSensorsMeasurement::D2, TwoSensorsMeasurement::D2) = 0.09;

//   sensor_left.setCovarianceSquareRoot(cov_left);
//   sensor_right.setCovarianceSquareRoot(cov_right);

//   std::cout << "REAL d_mid : " << right.d1() << std::endl
// 		<< "REAL d_right : " << right.d2() << std::endl;

//   std::cout << "REAL d_mid : " << left.d1() << std::endl
// 		<< "REAL d_left: " << left.d2() << std::endl;

//   // std::cin >> c;

//   if (i % 300 == 0 || i < 250) {

// 	std::cout << 2*sqrt(ekf.getCovariance()(State::X, State::X));
// 	char ch;
// 	//	std::cin >> ch;

//   }

//   //x_ekf = ekf.update(sensor_left, left, time, true, 1.5);
//   // x_ekf = ekf.update(sensor_right, right, time, true, 1.5);

//   x_ekf = ekf.update(sensor_left, left, time, true, 1);
//   x_ekf = ekf.update(sensor_right, right, time, true, 1);
// }
