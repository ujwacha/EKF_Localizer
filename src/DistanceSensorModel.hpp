#include "kalman/Matrix.hpp"
#include "kalman/StandardBase.hpp"
#include <cstddef>
#include <ostream>

#include "kalman/LinearizedMeasurementModel.hpp"
#include "Distance_Bet.hpp"

#define PI 3.1415

namespace Robot {
  template <typename T> class TwoSensorsMeasurement: public Kalman::Vector<T, 1> {
  public:
    KALMAN_VECTOR(TwoSensorsMeasurement, T, 1)

    static constexpr std::size_t D1 = 0;

    T d1() const { return (*this)[D1]; }
    T &d1() { return (*this)[D1]; }

  };

  template <typename T,
	    template <class> class CovarianceBase = Kalman::StandardBase>
  class TwoSensorsMeasurementModel
    : public Kalman::LinearizedMeasurementModel<State<T>, TwoSensorsMeasurement<T>, CovarianceBase> {
  public:
    //! State type shortcut definition
    typedef State<T> S;
    
    //! Measurement type shortcut definition
    typedef TwoSensorsMeasurement<T> M;

    mutable RobotSensorKalman<T> d1;

 
    TwoSensorsMeasurementModel(T angle1_, T r1_, T l_, T b_):
      d1(angle1_, r1_, l_, b_)
    {
      this->H.setIdentity();
      this->V.setIdentity();

    }
  
    M h(const S& x) const {
      M measurement;

      // std::cout << "KF THINKS: " << std::endl;

      // std::cout << "x: " << x.x() << std::endl
      // 		<< "y: " << x.y() << std::endl
      // 		<< "th: " << x.theta() << std::endl << std::endl;
      
      std::cout 
	<< "d1:" << d1.calculate(x.x(), x.y(), x.theta()).distance << std::endl
	<< std::endl;

      measurement.d1() = d1.calculate(x.x(), x.y(), x.theta()).distance;

      // char c;
      // std::cin >> c;

      return measurement;
    }

    void updateJacobians(const S& x, const double t = 0.05)
    {

      // std::cout << "MINI UPDATE" << std::endl;

      this->H.setZero();
      this->H(M::D1, S::X) = d1.calculate(x.x(), x.y(), x.theta()).dx;
      this->H(M::D1, S::Y) = d1.calculate(x.x(), x.y(), x.theta()).dy;
      this->H(M::D1, S::THETA) = d1.calculate(x.x(), x.y(), x.theta()).dtheta;
    }
  
  };
}
