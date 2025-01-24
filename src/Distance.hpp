#include <iostream>

#define PI 3.1415
enum Wall {
  B1,
  L1,
  B2,
  L2
};

template <typename T>
class Distance {
private:
  T angle_offset, radius;
  Wall w;
  T l, b;

  T tem_x, tem_y, tem_theta;
  

  void update_wall(T x, T y, T th) {
    T theta = std::atan(y/x);
    T alpha = std::atan(y/(b - x));
    T beta = std::atan((l-y)/(b - x));
    T sigma = std::atan((l-y)/x);

      if (th >= -alpha && th <= beta) w = Wall::L2;
      if (th >= beta && th <= PI - sigma) w = Wall::B2;
      if (th <= -alpha && th >= -(PI - theta)) w = Wall::B1;
      if ((th >= sigma && th <= PI) || (th >= -PI && th <= -theta)) w = Wall::L1;
      // If everythin fails
      w = Wall::L1;
  }
 
public:

  Distance(T angle_offset_, T radius_, T len, T bre):
    angle_offset(angle_offset_),
    radius(radius_),
    w(Wall::L1),
    l(len),
    b(bre) {}

 
  T get_distance(T x, T y, T theta) {
    theta = theta + angle_offset;
    update_wall(x, y, theta);

    tem_x = x;
    tem_y = y;
    tem_theta = theta;

    T d = 0;
    switch (w) {
    case Wall::L1:
      d = (-x)/std::cos(theta);
      break;
    case Wall::L2:
      d = (b - x)/std::cos(theta);
      break;
    case Wall::B1:
      d = (-y)/std::sin(theta);
      break;
    case Wall::B2:
      d = (l - y)/std::sin(theta);
    }
    return d;
  }

  T get_dx() {
    T d = 0;
    switch (w) {
    case Wall::L1:
      d = -1/std::cos(tem_theta);
      break;
    case Wall::L2:
      d = -1/std::cos(tem_theta);
      break;
    case Wall::B1:
      d = 0;
      break;
    case Wall::B2:
      d = 0;
    }

    return d;
  }

  T get_dy() {
    T d = 0;
    switch (w) {
    case Wall::L1:
      d = 0;
      break;
    case Wall::L2:
      d = 0;
      break;
    case Wall::B1:
      d = -1/std::sin(tem_theta);
      break;
    case Wall::B2:
      d = -1/std::sin(tem_theta);
    }

    return d;
  }

  T get_dth() {
    T d = 0;
    switch (w) {
    case Wall::L1:
      d = (-tem_x)*((std::sin(tem_theta))/(std::cos(tem_theta) * std::cos(tem_theta)));
      break;
    case Wall::L2:
      d = (b - tem_x) * ((std::sin(tem_theta))/(std::cos(tem_theta) * std::cos(tem_theta)));
      break;
    case Wall::B1:
      d = -tem_y*(-std::cos(tem_theta)/(std::sin(tem_theta) * std::sin(tem_theta)));
      break;
    case Wall::B2:
      d = (l -tem_y)*(-std::cos(tem_theta)/(std::sin(tem_theta) * std::sin(tem_theta)));
      break;
    }

    return d;
  }
};
