#include <cmath>
#include <tuple>
#include <iostream>

template <typename T> class RobotSensorKalman {
private:
  T l, b, r, a;       // Environment and robot parameters
  const T eps = 1e-6; // For numerical stability

  // Structure to hold both distance and derivatives
  struct SensorResult {
    T distance;
    T dx;     // ∂distance/∂x
    T dy;     // ∂distance/∂y
    T dtheta; // ∂distance/∂theta_s
  };

  T normalizeAngle(T angle) {
    while (angle > M_PI)
      angle -= 2 * M_PI;
    while (angle < -M_PI)
      angle += 2 * M_PI;
    return angle;
  }

public:
  RobotSensorKalman(T angle, T radius , T length , T breadth)
    : l(length), b(breadth), r(radius), a(angle) {}

  SensorResult calculate(T x, T y, T theta_s) {
    SensorResult result;
    theta_s = normalizeAngle(theta_s + a);

    // print_res(result);


    // std::cout << "Sensor{l: " << l << ", b: " << b << ", r: " << r << ", a: " << a <<  " } : " <<  std::endl;

    // std::cout << "DISTANCE{x: " << x << ", y: " << y << ", theta_s: " << theta_s << " } : " <<  std::endl;

    //Calculate angles to boundaries
    const T theta = std::atan2(y, x);
    const T beta = std::atan2(y, b - x);
    const T alpha = std::atan2(l - y, b - x);
    const T gamma = std::atan2(l - y, x);



    // Determine which edge we're facing
    if (-beta <= theta_s && theta_s <= alpha) { // Right edge
      result.distance = (b - x) / std::cos(theta_s) - r;
      result.dx = -1.0 / std::cos(theta_s);
      result.dy = 0.0;
      result.dtheta = (b - x) * std::sin(theta_s) /
                      (std::cos(theta_s) * std::cos(theta_s) + eps);
    } else if (alpha <= theta_s && theta_s <= M_PI - gamma) { // Top edge
      result.distance = (l - y) / std::sin(theta_s) - r;
      result.dx = 0.0;
      result.dy = -1.0 / std::sin(theta_s);
      result.dtheta = -(l - y) * std::cos(theta_s) /
                      (std::sin(theta_s) * std::sin(theta_s) + eps);
    } else if (-M_PI + theta <= theta_s && theta_s <= -beta) { // Bottom edge
      result.distance = -y / std::sin(theta_s) - r;
      result.dx = 0.0;
      result.dy = -1.0 / std::sin(theta_s);
      result.dtheta =
          y * std::cos(theta_s) / (std::sin(theta_s) * std::sin(theta_s) + eps);
    } else { // Left edge
      result.distance = -x / std::cos(theta_s) - r;
      result.dx = -1.0 / std::cos(theta_s);
      result.dy = 0.0;
      result.dtheta = -x * std::sin(theta_s) /
                      (std::cos(theta_s) * std::cos(theta_s) + eps);
    }

    print_res(result);

    // Clamp negative distances and handle numerical instability
    if (result.distance < 0) {
      result.distance = 0;
      result.dx = result.dy = result.dtheta = 0;
    }

    return result;
  }

  void print_res(SensorResult s) {
    
    // std::cout << "RESULT{dis: " << s.distance << ", dx: " << s.dx << ", dy: " << s.dy << ", dth: " << s.dtheta << " } : " <<  std::endl;
    
  }
};
