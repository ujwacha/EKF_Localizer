#include <iostream>

#include "SystemModel.hpp"
#include "OdomMeasurementModel.hpp"
#include "ImuMeasuremenntModel.hpp"


typedef float T;
typedef Robot::State<T> State;
typedef Robot::Twist<T> Twist;
typedef Robot::SystemModel<T> SystemModel;

typedef Robot::OdomMeasurement<T> OdomMeasurement;
typedef Robot::ImuMeasurement<T> ImuMeasurement;
typedef Robot::OdomMeasurementModel<T> OdomMeasurementModel;
typedef Robot::ImuMeasurementModel<T> ImuMeasurementModel;
		 


int main() {
  std::cout << "Hello World" << std::endl;

  State state;
  Twist twist;

  OdomMeasurementModel odom_model;
  ImuMeasurementModel imu_model;
  
  return 0;
}
