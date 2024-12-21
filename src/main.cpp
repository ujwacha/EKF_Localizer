#include <iostream>

#include <iostream>
#include <fstream>

#include "SystemModel.hpp"
#include "OdomMeasurementModel.hpp"
#include "ImuMeasuremenntModel.hpp"

#include "kalman/ExtendedKalmanFilter.hpp"


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

  //     // Random number generation (for noise simulation)
  // std::default_random_engine generator;
  // generator.seed( std::chrono::system_clock::now().time_since_epoch().count() );
  // std::normal_distribution<T> noise(0, 1);
  
 
  Kalman::ExtendedKalmanFilter<State> predictor;

  predictor.init(state);

  twist.rvx() = 1;
  twist.rvy() = 2;
  twist.romega() = 3;


  for (int i = 0; i < 300; i++) {
    state = sys.f(state, twist, 0.05);

    fs << i << "," << state.x() << "," << state.y() << "," << state.theta() << std::endl;
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
      twist.romega() = 0;
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
