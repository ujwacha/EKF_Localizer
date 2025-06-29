#include "kalman/Matrix.hpp"
#include "kalman/StandardBase.hpp"
#include <cstddef>
#include <ostream>

#include "kalman/LinearizedMeasurementModel.hpp"

#define PI 3.1415

namespace Robot {
template <typename T>
class FourSensorsMeasurement : public Kalman::Vector<T, 4> {
public:
  KALMAN_VECTOR(FourSensorsMeasurement, T, 4)

  static constexpr std::size_t D1 = 0;
  static constexpr std::size_t D2 = 1;
  static constexpr std::size_t D3 = 2;
  static constexpr std::size_t D4 = 3;

  T d1() const { return (*this)[D1]; }
  T &d1() { return (*this)[D1]; }

  T d2() const { return (*this)[D2]; }
  T &d2() { return (*this)[D2]; }

  T d3() const { return (*this)[D3]; }
  T &d3() { return (*this)[D3]; }

  T d4() const { return (*this)[D4]; }
  T &d4() { return (*this)[D4]; }
};

template <typename T,
          template <class> class CovarianceBase = Kalman::StandardBase>
class FourDistanceSensorsMeasumementModel
    : public Kalman::LinearizedMeasurementModel<
          State<T>, FourSensorsMeasurement<T>, CovarianceBase> {
public:
  //! State type shortcut definition
  typedef State<T> S;

  //! Measurement type shortcut definition
  typedef FourSensorsMeasurement<T> M;

  mutable RobotSensorKalman<T> d1, d2, d3, d4;

  FourDistanceSensorsMeasumementModel(T angle1_, T r1_, T angle_b1, T angle2_,
                                      T r2_, T angle_b2, T angle3_, T r3_,
                                      T angle_b3, T angle4_, T r4_, T angle_b4,
                                      T l_, T b_)
      : d1(angle1_, r1_, angle_b1, l_, b_),
	d2(angle2_, r2_, angle_b2, l_, b_),
        d3(angle3_, r3_, angle_b3, l_, b_),
	d4(angle4_, r4_, angle_b4, l_, b_)
  {
    this->H.setIdentity();
    this->V.setIdentity();
  }

  M h(const S &x) const {
    M measurement;

#ifdef PRINT_DEBUG
    std::cout << "KF THINKS: " << std::endl;

    std::cout << "x: " << x.x() << std::endl
              << "y: " << x.y() << std::endl
              << "th: " << x.theta() << std::endl
              << std::endl;

    std::cout << "d1:" << d1.calculate(x.x(), x.y(), x.theta()).distance
              << std::endl
              << "d2:" << d2.calculate(x.x(), x.y(), x.theta()).distance
              << std::endl
              << "d3:" << d3.calculate(x.x(), x.y(), x.theta()).distance
              << std::endl
              << "d4:" << d4.calculate(x.x(), x.y(), x.theta()).distance
              << std::endl
              << std::endl;
#endif

    measurement.d1() = d1.calculate(x.x(), x.y(), x.theta()).distance;
    measurement.d2() = d2.calculate(x.x(), x.y(), x.theta()).distance;
    measurement.d3() = d3.calculate(x.x(), x.y(), x.theta()).distance;
    measurement.d4() = d4.calculate(x.x(), x.y(), x.theta()).distance;

    return measurement;
  }

  void updateJacobians(const S &x, const double t = 0.05) {

    // std::cout << "MINI UPDATE" << std::endl;

    this->H.setZero();
    this->H(M::D1, S::X) = d1.calculate(x.x(), x.y(), x.theta()).dx;
    this->H(M::D1, S::Y) = d1.calculate(x.x(), x.y(), x.theta()).dy;
    this->H(M::D1, S::THETA) = d1.calculate(x.x(), x.y(), x.theta()).dtheta;

    this->H(M::D2, S::X) = d2.calculate(x.x(), x.y(), x.theta()).dx;
    this->H(M::D2, S::Y) = d2.calculate(x.x(), x.y(), x.theta()).dy;
    this->H(M::D2, S::THETA) = d2.calculate(x.x(), x.y(), x.theta()).dtheta;

    this->H(M::D3, S::X) = d3.calculate(x.x(), x.y(), x.theta()).dx;
    this->H(M::D3, S::Y) = d3.calculate(x.x(), x.y(), x.theta()).dy;
    this->H(M::D3, S::THETA) = d3.calculate(x.x(), x.y(), x.theta()).dtheta;

    this->H(M::D4, S::X) = d4.calculate(x.x(), x.y(), x.theta()).dx;
    this->H(M::D4, S::Y) = d4.calculate(x.x(), x.y(), x.theta()).dy;
    this->H(M::D4, S::THETA) = d4.calculate(x.x(), x.y(), x.theta()).dtheta;

    double value = 1.5;

    if ((fabs(this->H(M::D1, S::X)) + fabs(this->H(M::D1, S::Y))) > value)
      this->V(M::D1, M::D1) = 600;
    else
      this->V(M::D1, M::D1) = 1;

    if ((fabs(this->H(M::D2, S::X)) + fabs(this->H(M::D2, S::Y))) > value)
      this->V(M::D2, M::D2) = 600;
    else
      this->V(M::D2, M::D2) = 1;

    if ((fabs(this->H(M::D3, S::X)) + fabs(this->H(M::D3, S::Y))) > value)
      this->V(M::D3, M::D3) = 600;
    else
      this->V(M::D3, M::D3) = 1;

    if ((fabs(this->H(M::D4, S::X)) + fabs(this->H(M::D4, S::Y))) > value)
      this->V(M::D4, M::D4) = 600;
    else
      this->V(M::D4, M::D4) = 1;
  }
};
} // namespace Robot
