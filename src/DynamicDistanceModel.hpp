#include <cstdint>
#define SENSOR_ONE_RADIUS 0.295
#define SENSOR_TWO_RADIUS 0.273
#define SENSOR_THREE_RADIUS 0.275
#define SENSOR_FOUR_RADIUS 0.265

#define SENSOR_ONE_ANGLE 0
#define SENSOR_TWO_ANGLE 3.141592 / 2
#define SENSOR_THREE_ANGLE 3.141592
#define SENSOR_FOUR_ANGLE -3.141592 / 2

enum SensorMasks {
  SENSOR_ONE = 1 << 3,
  SENSOR_TWO = 1 << 2,
  SENSOR_THREE = 1 << 1,
  SENSOR_FOUR = 1 << 0
};

TFMiniMeasurementModel get_sensor_three(unsigned int mask_value) {

  switch (mask_value) {
  case (SENSOR_TWO | SENSOR_THREE | SENSOR_FOUR):
    return TFMiniMeasurementModel(SENSOR_TWO_ANGLE, SENSOR_TWO_RADIUS, SENSOR_TWO_ANGLE,
                                  SENSOR_THREE_ANGLE, SENSOR_THREE_RADIUS, SENSOR_THREE_ANGLE,
                                  SENSOR_FOUR_ANGLE, SENSOR_FOUR_RADIUS, SENSOR_FOUR_ANGLE, 8, 15);
    break;

  case (SENSOR_ONE | SENSOR_THREE | SENSOR_FOUR):
    return TFMiniMeasurementModel(SENSOR_ONE_ANGLE, SENSOR_ONE_RADIUS, SENSOR_ONE_ANGLE,
                                  SENSOR_THREE_ANGLE, SENSOR_THREE_RADIUS, SENSOR_THREE_ANGLE,
                                  SENSOR_FOUR_ANGLE, SENSOR_FOUR_RADIUS, SENSOR_FOUR_ANGLE, 8, 15);
    break;

  case (SENSOR_ONE | SENSOR_TWO | SENSOR_FOUR):
    return TFMiniMeasurementModel(SENSOR_ONE_ANGLE, SENSOR_ONE_RADIUS, SENSOR_ONE_ANGLE,
                                  SENSOR_TWO_ANGLE, SENSOR_TWO_RADIUS, SENSOR_TWO_ANGLE,
                                  SENSOR_FOUR_ANGLE, SENSOR_FOUR_RADIUS, SENSOR_FOUR_ANGLE, 8, 15);
    break;

  case (SENSOR_ONE | SENSOR_TWO | SENSOR_THREE):
    return TFMiniMeasurementModel(SENSOR_ONE_ANGLE, SENSOR_ONE_RADIUS, SENSOR_ONE_ANGLE,
				  SENSOR_TWO_ANGLE, SENSOR_TWO_RADIUS, SENSOR_TWO_ANGLE,
				  SENSOR_THREE_ANGLE, SENSOR_THREE_RADIUS, SENSOR_THREE_ANGLE, 8, 15);
    break;
  default:
    std::cout << "You sent the wrong mask value" << std::endl;
    char ccc;
    std::cin >> ccc;
    return TFMiniMeasurementModel(SENSOR_ONE_ANGLE, SENSOR_ONE_RADIUS, SENSOR_ONE_ANGLE,
				  SENSOR_TWO_ANGLE, SENSOR_TWO_RADIUS, SENSOR_TWO_ANGLE,
				  SENSOR_THREE_ANGLE, SENSOR_THREE_RADIUS, SENSOR_THREE_ANGLE, 8, 15);
    break;

  }
}

DistanceSensorMeasurementModel get_sensor_one(unsigned int mask) {

  switch (mask) {

  case SENSOR_ONE:
    return DistanceSensorMeasurementModel(SENSOR_ONE_ANGLE, SENSOR_ONE_RADIUS, SENSOR_ONE_ANGLE,
                                          8, 15);
    break;

  case SENSOR_TWO:
    return DistanceSensorMeasurementModel(SENSOR_TWO_ANGLE, SENSOR_TWO_RADIUS, SENSOR_TWO_ANGLE,
                                          8, 15);
    break;

  case SENSOR_THREE:
    return DistanceSensorMeasurementModel(SENSOR_THREE_ANGLE, SENSOR_THREE_RADIUS, SENSOR_THREE_ANGLE,
					  8, 15);
    break;

  case SENSOR_FOUR:
    return DistanceSensorMeasurementModel(SENSOR_FOUR_ANGLE, SENSOR_FOUR_RADIUS, SENSOR_FOUR_ANGLE,
                                          8, 15);
    break;
  default:
    std::cout << "You sent the wrong mask value" << std::endl;
    char ccc;
    std::cin >> ccc;
    return DistanceSensorMeasurementModel(SENSOR_FOUR_ANGLE, SENSOR_FOUR_RADIUS, SENSOR_FOUR_ANGLE,
                                          8, 15);
    break;
  }
}

TwoSensorsMeasurementModel get_sensor_two(unsigned int mask) {

  switch (mask) {

  case (SENSOR_THREE | SENSOR_FOUR):
    return TwoSensorsMeasurementModel(SENSOR_THREE_ANGLE, SENSOR_THREE_RADIUS, SENSOR_THREE_ANGLE,
				      SENSOR_FOUR_ANGLE, SENSOR_FOUR_RADIUS, SENSOR_FOUR_ANGLE,
				      8, 15);
    break;

  case (SENSOR_TWO | SENSOR_FOUR):
    return TwoSensorsMeasurementModel(SENSOR_TWO_ANGLE, SENSOR_TWO_RADIUS, SENSOR_TWO_ANGLE,
				      SENSOR_FOUR_ANGLE, SENSOR_FOUR_RADIUS, SENSOR_FOUR_ANGLE,
				      8, 15);
    break;

  case (SENSOR_TWO | SENSOR_THREE):
    return TwoSensorsMeasurementModel(SENSOR_TWO_ANGLE, SENSOR_TWO_RADIUS, SENSOR_TWO_ANGLE,
				      SENSOR_THREE_ANGLE, SENSOR_THREE_RADIUS, SENSOR_THREE_ANGLE,
				      8, 15);
    break;

  case (SENSOR_ONE | SENSOR_FOUR):
    return TwoSensorsMeasurementModel(SENSOR_ONE_ANGLE, SENSOR_ONE_RADIUS, SENSOR_ONE_ANGLE,
				      SENSOR_FOUR_ANGLE, SENSOR_FOUR_RADIUS, SENSOR_FOUR_ANGLE,
				      8, 15);
    break;

  case (SENSOR_ONE | SENSOR_THREE):
    return TwoSensorsMeasurementModel(SENSOR_ONE_ANGLE, SENSOR_ONE_RADIUS, SENSOR_ONE_ANGLE,
				      SENSOR_THREE_ANGLE, SENSOR_THREE_RADIUS, SENSOR_THREE_ANGLE,
				      8, 15);
    break;



  case (SENSOR_ONE | SENSOR_TWO):
    return TwoSensorsMeasurementModel(SENSOR_ONE_ANGLE, SENSOR_ONE_RADIUS, SENSOR_ONE_ANGLE,
				      SENSOR_TWO_ANGLE, SENSOR_TWO_RADIUS, SENSOR_TWO_ANGLE,
				      8, 15);
    break;

  default:
    std::cout << "You sent the wrong mask value" << std::endl;
    char ccc;
    std::cin >> ccc;
    
    return TwoSensorsMeasurementModel(SENSOR_ONE_ANGLE, SENSOR_ONE_RADIUS, SENSOR_ONE_ANGLE,
				      SENSOR_TWO_ANGLE, SENSOR_TWO_RADIUS, SENSOR_TWO_ANGLE,
				      8, 15);
    break;
  }
}

FourDistanceSensorsMeasumementModel get_sensor_four() {
  return FourDistanceSensorsMeasumementModel(SENSOR_ONE_ANGLE, SENSOR_ONE_RADIUS, SENSOR_ONE_ANGLE,
					     SENSOR_TWO_ANGLE, SENSOR_TWO_RADIUS, SENSOR_TWO_ANGLE,
					     SENSOR_THREE_ANGLE, SENSOR_THREE_RADIUS, SENSOR_THREE_ANGLE,
					     SENSOR_FOUR_ANGLE, SENSOR_FOUR_RADIUS, SENSOR_FOUR_ANGLE,
					     8, 15);
}
