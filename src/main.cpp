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
#include "LowPassFilter.hpp"


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

typedef Robot::OdomMeasurementModel<T> OdomMeasurementModel;
typedef Robot::ImuMeasurementModel<T> ImuMeasurementModel;
typedef Robot::WheelMeasurementModel<T> WheelMeasurementModel;
typedef Robot::TFMiniMeasurementModel<T> TFMiniMeasurementModel;


template <typename T>
T degreesToRadians(T degrees) {
  return degrees * static_cast<T>(M_PI) / static_cast<T>(180);
}


double get_cov(double val) {
  //  return (950 * 0.4 /(val - 65));

  if (val < 850) return 70;
  else return 0.2;
}

int main() {
  // Open a file To store the csv file

  std::ofstream fs("../data/data.csv", std::ios::out);

  std::cout << "Hello Kalman Filter" << std::endl;

  State state;
  state.setZero();

  state.x() = 4.7;
  state.y() = 3.11;

  Twist twist;


  OdomMeasurementModel odom_model;
  ImuMeasurementModel imu_model;
  WheelMeasurementModel wheel_model(LEFT_RADIUS, RIGHT_RADIUS, MID_RADIUS, WHEEL_D / 2);
  SystemModel sys;
  TFMiniMeasurementModel mini(0, 0.3, PI/2, 0.3, -PI/2, 0.3, 4, 7.5);
  //TFMiniMeasurementModel mini(PI/2, 0.2, 0, 0.2, -PI/2, 0.2, 4, 7.5);

  Kalman::Covariance<OdomMeasurement> odom_cov;
  Kalman::Covariance<ImuMeasurement> imu_cov;
  Kalman::Covariance<WheelMeasurement> wheel_cov;
  Kalman::Covariance<State> state_cov;
  Kalman::Covariance<TFMiniMeasurement> tf_cov;

  
  state_cov.setIdentity();

  state_cov(State::VX, State::VX) = 1;
  state_cov(State::VY, State::VY) = 1;
  state_cov(State::OMEGA, State::OMEGA) = 1;

  wheel_cov(WheelMeasurement::OMEGA_L, WheelMeasurement::OMEGA_L) = 1;
  wheel_cov(WheelMeasurement::OMEGA_R, WheelMeasurement::OMEGA_L) = 1;
  wheel_cov(WheelMeasurement::OMEGA_M, WheelMeasurement::OMEGA_L) = 1;


  tf_cov(TFMiniMeasurement::D1, TFMiniMeasurement::D1) = 0.1;
  tf_cov(TFMiniMeasurement::D2, TFMiniMeasurement::D2) = 0.1;
  tf_cov(TFMiniMeasurement::D3, TFMiniMeasurement::D3) = 0.1;


  imu_cov(ImuMeasurement::AX, ImuMeasurement::AX) = 0.7;
  imu_cov(ImuMeasurement::AY, ImuMeasurement::AY) = 0.7;
  imu_cov(ImuMeasurement::YAW, ImuMeasurement::YAW) = 0.001;


  // odom_cov(OdomMeasurement::X, OdomMeasurement::X) = 100;
  // odom_cov(OdomMeasurement::Y, OdomMeasurement::Y) = 100;
  // odom_cov(OdomMeasurement::THETA, OdomMeasurement::THETA) = 0.1;

  // odom_cov(OdomMeasurement::VX, OdomMeasurement::VX) = 0.000625;
  // odom_cov(OdomMeasurement::VY, OdomMeasurement::VY) = 0.003;

  odom_cov(OdomMeasurement::VX, OdomMeasurement::VX) = 0.00000000625;
  odom_cov(OdomMeasurement::VY, OdomMeasurement::VY) = 0.00000003;



  odom_cov(OdomMeasurement::OMEGA, OdomMeasurement::OMEGA) = 0.001;

  odom_model.setCovariance(odom_cov);
  imu_model.setCovariance(imu_cov);
  wheel_model.setCovariance(wheel_cov);
  mini.setCovariance(tf_cov);
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



  std::vector iy  = csv.GetColumn<float>(12);
  std::vector iax = csv.GetColumn<float>(13);
  std::vector iay = csv.GetColumn<float>(14);

  std::vector dm = csv.GetColumn<float>(16);
  std::vector dr = csv.GetColumn<float>(17);
  std::vector dl = csv.GetColumn<float>(18);

  std::vector sm = csv.GetColumn<float>(19);
  std::vector sr = csv.GetColumn<float>(20);
  std::vector sl = csv.GetColumn<float>(21);



  char c;

  double time = 0.01;

  LowPassFilter low_pass_right(5.0, 0.01f);
  LowPassFilter low_pass_left(5.0, 0.01f);
  LowPassFilter low_pass_mid(5.0, 0.01f);

  for (int i = 00; i < rx.size() ; i++) {

    time = t[i] - t[i-1];

    // state = sys.f(state, twist, time);

    auto x_ekf = ekf.predict(sys, twist, time);
    auto x_pred = predictor.predict(sys, twist, time);

    // Wheel Measurement
    // {
    //   WheelMeasurement wheel;

    //   wheel.omega_r() = o_r[i];
    //   wheel.omega_l() = o_m[i];
    //   wheel.omega_m() = o_l[i];

    //   // std::cout << "WHEEEEELLLLL    " << std::endl;
    //   // std::cout << "or: " << o_r[i] << " om: " << o_m[i] << "ol: " << o_l[i] << std::endl;
    //   // std::cin >> c;

    //   x_ekf = ekf.update(wheel_model, wheel, time);

    // }
    // Odom 
    {
      // double omega_r = low_pass_right.update(o_r[i]);
      // double omega_l = low_pass_left.update(o_m[i]);
      // double omega_m = low_pass_mid.update(o_l[i]);

      double omega_r = o_r[i];
      double omega_l = o_l[i];
      double omega_m = o_m[i];

      double back_vel = omega_m * WHEEL_D / 2.0f;
      double right_vel = omega_r * WHEEL_D / 2.0f;
      double left_vel = omega_l * WHEEL_D / 2.0f;
      
      double omega = (right_vel - left_vel) / (RIGHT_RADIUS + LEFT_RADIUS);
      double vx = (right_vel * LEFT_RADIUS + left_vel * RIGHT_RADIUS) / (RIGHT_RADIUS + LEFT_RADIUS);
      double vy = back_vel - omega * MID_RADIUS; // might have to change it to + or minus for better results

      OdomMeasurement mea;
      
      mea.vx() = vx;
      mea.vy() = vy;
      mea.omega() = omega;

      // if (i % 100 == 0) {

	// std::cout << "MEASSS " << std::endl << std::endl << "vx: "  << vx << std::endl << "vy: " << vy << std::endl << "omega: " << omega << std::endl << "ENDDDD" << std::endl;

	//	std::cin >> c;
      std::cout << vx << " , " << vy << " , "<< omega << std::endl;
	
      // }

      x_ekf = ekf.update(odom_model, mea, time);
    }

    // Imu Measurement
    {
      // We can measure the orientation every 5th step
      ImuMeasurement imu;
      // Measurement is affected by noise as well
      imu.yaw() = iy[i] ;
      imu.ax() = iax[i];
      imu.ay() = iay[i];

      // Update EK// F
      x_ekf = ekf.update(imu_model, imu, time);
    }



    {
      TFMiniMeasurement min;
      min.d1() = dm[i]/100;
      min.d3() = dl[i]/100;
      min.d2() = dr[i]/100;



      tf_cov(TFMiniMeasurement::D1, TFMiniMeasurement::D1) = get_cov(sm[i]);
      tf_cov(TFMiniMeasurement::D3, TFMiniMeasurement::D3) = get_cov(sl[i]);
      tf_cov(TFMiniMeasurement::D2, TFMiniMeasurement::D2) = get_cov(sr[i]);

      mini.setCovarianceSquareRoot(tf_cov);


      // std::cout << "REAL d1 : " << min.d1() << std::endl
      // 		<< "REAL d2 : " << min.d2() << std::endl
      // 		<< "REAL d3 : " << min.d3() << std::endl;


      //if (i % 300 == 0 || i < 250) 
	x_ekf = ekf.update(mini, min, time);

    }

    fs << i << "," << x_ekf.vx() << "," << x_ekf.vy() << ","<< state.theta() << ","
       << x_ekf.x() << "," << x_ekf.y() << "," << x_ekf.theta() << ","
       << x_pred.x() << "," << x_pred.y() << "," << x_pred.theta() 
       << "," << ox[i] << "," << oy[i] << "," << oth[i] << "," << iy[i] << std::endl;
    
    
    
    
    twist.romega() = rw[i];
    twist.rvx() = rx[i];
    twist.rvy() = ry[i];

  }


  // check if this works properly
  
  
  return 0;
}
