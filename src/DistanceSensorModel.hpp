#include "kalman/Matrix.hpp"
#include "kalman/StandardBase.hpp"
#include <cstddef>
#include <ostream>

#include "kalman/LinearizedMeasurementModel.hpp"
//// /#include "Distance_Bet.hpp"


#define PI 3.1415

namespace Robot {
  template <typename T> class DistanceSensorMeasurement: public Kalman::Vector<T, 1> {
  public:
    KALMAN_VECTOR(DistanceSensorMeasurement, T, 1)

    static constexpr std::size_t D1 = 0;

    T d1() const { return (*this)[D1]; }
    T &d1() { return (*this)[D1]; }

  };

  template <typename T,
	    template <class> class CovarianceBase = Kalman::StandardBase>
  class DistanceSensorMeasurementModel
    : public Kalman::LinearizedMeasurementModel<State<T>, DistanceSensorMeasurement<T>, CovarianceBase> {
  public:
    //! State type shortcut definition
    typedef State<T> S;
    
    //! Measurement type shortcut definition
    typedef DistanceSensorMeasurement<T> M;

    mutable RobotSensorKalman<T> d1;

 
    DistanceSensorMeasurementModel(T angle1_, T r1_, T angle_b1, T l_, T b_):
      d1(angle1_, r1_, angle_b1, l_, b_)
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

      std::cout << "H: " << this->H(M::D1, S::THETA) << std::endl;

      // if (fabs(x.vx()) < 0.15 && fabs(x.vy()) < 0.15 && fabs(x.omega()) > 0.8)
      //   this->V(M::D1, M::D1) = 10;
      // else
      //   this->V(M::D1, M::D1) = 1;


      // angles you want to reject is given by
      //cos (angle) = 1/value 

      double value = 2;
      
      if ((fabs(this->H(M::D1, S::X)) + fabs(this->H(M::D1, S::Y))) > value)
	this->V(M::D1, M::D1) = 6000;
      else 
	this->V(M::D1, M::D1) = 1;

      // this->V(M::D1, M::D1) = 1 + this->H(M::D1, S::THETA) * 2;

    }
  
  };
}
