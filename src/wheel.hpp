#include <iostream>


template <typename T>
class Encoder {
private:
  T x, y , a, b, r;

  struct Data {
    T omega;
    T dtheta;
    T dvx;
    T dvy;
    T domega;
  };

public:

  Data result;

  Encoder(T x_, T y_, T alpha_, T radius_) {
    this->x = x_;
    this->y = y_;
    this->a = std::cos(alpha_); // alpha_ is the angle the encoder wheel makes with the x axis 
    this->b = std::sin(alpha_);
    r = radius_;
  }

  Data calculate(T theta, T vx, T vy, T omega) {
    result.omega = ((a*std::cos(theta) - b*std::sin(theta))*vx
		    + (a*std::sin(theta) + b*std::cos(theta))*vy
		    - a*y*omega
		    + b*x*omega)/r;

    result.dvx = (a*std::cos(theta) - b*std::sin(theta))/r;
    result.dvy = (a*std::sin(theta) + b*std::cos(theta))/r;
    result.domega = (b*x - a*y)/r;
    result.dtheta = ((-a*std::sin(theta) - b*std::cos(theta))*vx + (a*std::cos(theta) - b*std::sin(theta))*vy)/r;

    return result;
  }
};
