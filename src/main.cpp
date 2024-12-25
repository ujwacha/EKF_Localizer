#include <iostream>
#include <iostream>
#include <random>
#include <chrono>
#include <fstream>
#include <vector>

#include "SystemModel.hpp"
#include "OdomMeasurementModel.hpp"
#include "ImuMeasuremenntModel.hpp"
#include "kalman/ExtendedKalmanFilter.hpp"

#include "kalman/Types.hpp"
#include "rapidcsv.h"



typedef double T;

typedef Robot::State<T> State;
typedef Robot::Twist<T> Twist;
typedef Robot::SystemModel<T> SystemModel;

typedef Robot::OdomMeasurement<T> OdomMeasurement;
typedef Robot::ImuMeasurement<T> ImuMeasurement;

typedef Robot::OdomMeasurementModel<T> OdomMeasurementModel;
typedef Robot::ImuMeasurementModel<T> ImuMeasurementModel;


template <typename T>
T degreesToRadians(T degrees) {
  return degrees * static_cast<T>(M_PI) / static_cast<T>(180);
}

		 

int main() {
  // Open a file To store the csv file

  std::ofstream fs("../data/data.csv", std::ios::out);

  std::cout << "Hello Kalman Filter" << std::endl;

  State state;
  state.setZero();

  Twist twist;


  OdomMeasurementModel odom_model;
  ImuMeasurementModel imu_model;
  SystemModel sys;

  Kalman::Covariance<OdomMeasurement> odom_cov;
  Kalman::Covariance<ImuMeasurement> imu_cov;

  // imu_cov(ImuMeasurement::AX, ImuMeasurement::AX) = 2;
  // imu_cov(ImuMeasurement::AY, ImuMeasurement::AY) = 2;
  // imu_cov(ImuMeasurement::YAW, ImuMeasurement::YAW) = 0.1;


  // odom_cov(OdomMeasurement::X, OdomMeasurement::X) = 5;
  // odom_cov(OdomMeasurement::Y, OdomMeasurement::Y) = 5;
  // odom_cov(OdomMeasurement::THETA, OdomMeasurement::THETA) = 10;
  // odom_cov(OdomMeasurement::VX, OdomMeasurement::VX) = 1;
  // odom_cov(OdomMeasurement::VY, OdomMeasurement::VY) = 1;
  // odom_cov(OdomMeasurement::OMEGA, OdomMeasurement::OMEGA) = 3;

  // odom_model.setCovariance(odom_cov);
  // imu_model.setCovariance(imu_cov);

  /// Set Covariance Of State Model



 
 
  Kalman::ExtendedKalmanFilter<State> predictor;
  Kalman::ExtendedKalmanFilter<State> ekf;

  predictor.init(state);
  ekf.init(state);



  // twist.rvx() = 1;
  // twist.rvy() = 2;
  // twist.romega() = 3;

  // Get The CSV File

  rapidcsv::Document csv("../real_life_dta/robot_data.csv");

  std::vector rx = csv.GetColumn<float>(2);
  std::vector ry = csv.GetColumn<float>(3);
  std::vector rw = csv.GetColumn<float>(4);
  std::vector t = csv.GetColumn<float>(1);

  std::vector ox = csv.GetColumn<float>(5);
  std::vector oy = csv.GetColumn<float>(6);
  std::vector oth = csv.GetColumn<float>(7);
  std::vector ovx = csv.GetColumn<float>(8);
  std::vector ovy = csv.GetColumn<float>(9);
  std::vector ow = csv.GetColumn<float>(10);


  std::vector iy  = csv.GetColumn<float>(13);
  std::vector iax = csv.GetColumn<float>(14);
  std::vector iay = csv.GetColumn<float>(15);



  float time = 0.01;

  for (int i = 0; i < rx.size() ; i++) {

    

    // state = sys.f(state, twist, time);

    auto x_ekf = ekf.predict(sys, twist, time);
    auto x_pred = predictor.predict(sys, twist, time);


    // Odom measurement
    {
      // We can measure the orientation every 5th step
      OdomMeasurement odom;
      // Measurement is affected by noise as well
      odom.theta() = oth[i];
      odom.omega() = ow[i];
      odom.x() = ox[i];
      odom.y() = oy[i];
      odom.vx() = ovx[i];
      odom.vy() = ovy[i];
      
      // Update EKF
      x_ekf = ekf.update(odom_model, odom, time);
            
    }

    // Imu Measurement
    {
      // We can measure the orientation every 5th step
      ImuMeasurement imu;
      // Measurement is affected by noise as well
      imu.yaw() = degreesToRadians(iy[i]);
      imu.ax() = iax[i];
      imu.ay() = iay[i];

      // Update EKF
      x_ekf = ekf.update(imu_model, imu, time);
            
    }



    fs << i << "," << state.x() << "," << state.y() << ","<< state.theta() << ","
       << x_ekf.x() << "," << x_ekf.y() << "," << x_ekf.theta() << ","
       << x_pred.x() << "," << x_pred.y() << "," << x_pred.theta() 
       << "," << ox[i] << "," << oy[i] << "," << oy[i] << std::endl;


    twist.romega() = rw[i];
    twist.rvx() = rx[i];
    twist.rvy() = ry[i];

  }


  // check if this works properly
  
  
  return 0;
}
