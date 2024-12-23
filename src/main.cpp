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

#include "rapidcsv.h"



typedef float T;

typedef Robot::State<T> State;
typedef Robot::Twist<T> Twist;
typedef Robot::SystemModel<T> SystemModel;

typedef Robot::OdomMeasurement<T> OdomMeasurement;
typedef Robot::ImuMeasurement<T> ImuMeasurement;

typedef Robot::OdomMeasurementModel<T> OdomMeasurementModel;
typedef Robot::ImuMeasurementModel<T> ImuMeasurementModel;
		 

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


  T SystemNoise = 0.03;
  T OdomNoise = 0.1;
  T ImuNoise = 0.06;
    

      // Random number generation (for noise simulation)
  std::default_random_engine generator;
  generator.seed( std::chrono::system_clock::now().time_since_epoch().count() );
  std::normal_distribution<T> noise(0, 1);
  
 
  Kalman::ExtendedKalmanFilter<State> predictor;
  Kalman::ExtendedKalmanFilter<State> ekf;

  predictor.init(state);
  ekf.init(state);



  twist.rvx() = 1;
  twist.rvy() = 2;
  twist.romega() = 3;

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

    

    state = sys.f(state, twist, time);



    // state.x() += SystemNoise*noise(generator);
    // state.y() += SystemNoise*noise(generator);
    // state.theta() += SystemNoise*noise(generator);
    // state.vx() += SystemNoise*noise(generator);
    // state.vy() += SystemNoise*noise(generator);
    // state.ax() += SystemNoise*noise(generator);
    // state.ay() += SystemNoise*noise(generator);


    auto x_ekf = ekf.predict(sys, twist, time);
    auto x_pred = predictor.predict(sys, twist, time);
    
    


        // Odom measurement
        {
            // We can measure the orientation every 5th step
            OdomMeasurement odom = odom_model.h(state);
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
            ImuMeasurement imu = imu_model.h(state);
            // Measurement is affected by noise as well
            imu.yaw() = iy[i];
            imu.ax() = iax[i];
            imu.ay() = iay[i];

            // Update EKF
            x_ekf = ekf.update(imu_model, imu, time);
            
        }



	fs << i << "," << state.x() << "," << state.y() << ","<< state.theta() << ","
	   << x_ekf.x() << "," << x_ekf.y() << "," << x_ekf.theta() << ","
	   << x_pred.x() << "," << x_pred.y() << "," << x_pred.theta() << std::endl;


	twist.romega() = rw[i];
	twist.rvx() = rx[i];
	twist.rvy() = ry[i];

  }


  // check if this works properly
  
  
  return 0;
}
