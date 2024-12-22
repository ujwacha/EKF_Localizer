#include <iostream>

#include <iostream>
#include <fstream>

#include "SystemModel.hpp"
#include "OdomMeasurementModel.hpp"
#include "ImuMeasuremenntModel.hpp"

#include "kalman/ExtendedKalmanFilter.hpp"

#include <iostream>
#include <random>
#include <chrono>



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


  for (int i = 0; i < 300; i++) {
    state = sys.f(state, twist, 0.05);

    state.x() += SystemNoise*noise(generator);
    state.y() += SystemNoise*noise(generator);
    state.theta() += SystemNoise*noise(generator);
    state.vx() += SystemNoise*noise(generator);
    state.vy() += SystemNoise*noise(generator);
    state.ax() += SystemNoise*noise(generator);
    state.ay() += SystemNoise*noise(generator);


    auto x_ekf = ekf.predict(sys, twist, 0.05);
    auto x_pred = predictor.predict(sys, twist, 0.05);




        // Odom measurement
        {
            // We can measure the orientation every 5th step
            OdomMeasurement odom = odom_model.h(state);
            // Measurement is affected by noise as well
            odom.theta() += OdomNoise * noise(generator);
            odom.omega() += OdomNoise * noise(generator);
            odom.x() += OdomNoise * noise(generator);
            odom.y() += OdomNoise * noise(generator);
            odom.vx() += OdomNoise * noise(generator);
            odom.vy() += OdomNoise * noise(generator);

            // Update EKF
            x_ekf = ekf.update(odom_model, odom, 0.05);
            
        }

	// Imu Measurement
        {
            // We can measure the orientation every 5th step
            ImuMeasurement imu = imu_model.h(state);
            // Measurement is affected by noise as well
            imu.yaw() += ImuNoise * noise(generator);
            imu.ax() += ImuNoise * noise(generator);
            imu.ay() += ImuNoise * noise(generator);

            // Update EKF
            x_ekf = ekf.update(imu_model, imu, 0.05);
            
        }



	fs << i << "," << state.x() << "," << state.y() << "," << state.theta() << "," << x_ekf.x() << "," << x_ekf.y() << "," << x_ekf.theta() << std::endl;
    // Should Have Used Case Statement Lol
    if (i  == 70) {
      std::cout << "OPP" << std::endl;
      twist.romega() = -3;
    }


    if (i  == 95) {
      std::cout << "OPP" << std::endl;
      twist.romega() = 0;
    }

    if (i == 120) {
      std::cout << "OPP" << std::endl;
      twist.romega() = 10;
    }

    if ( i==150 ) {
      twist.rvx() = 0;
      twist.rvy() = 0;
      twist.romega() = 1;
    }

    if (i==200) {
      twist.rvx() = 1;
    }

    if (i==240) {
      twist.rvx() = 1;
      twist.rvy() = 2;
      twist.romega() = 3;
    }



    //    std::cout << i << "," << state.x() << "," << state.y() << "," << state.theta() << std::endl;
  }


  // check if this works properly
  
  
  return 0;
}
