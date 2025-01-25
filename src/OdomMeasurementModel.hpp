#include "kalman/Matrix.hpp"
#include "kalman/StandardBase.hpp"
#include <cstddef>

#include "kalman/LinearizedMeasurementModel.hpp"

namespace Robot {
  

  template <typename T> class OdomMeasurement : public Kalman::Vector<T, 3> {
  public:
    KALMAN_VECTOR(OdomMeasurement, T, 3)

    static constexpr std::size_t VX = 0;
    static constexpr std::size_t VY = 1;
    static constexpr std::size_t OMEGA = 2;

    T vx() const { return (*this)[VX]; }
    T &vx() { return (*this)[VX]; }

    T vy() const { return (*this)[VY]; }
    T &vy() { return (*this)[VY]; }

    T omega() const { return (*this)[OMEGA]; }
    T &omega() { return (*this)[OMEGA]; }
  };

  template <typename T,
	    template <class> class CovarianceBase = Kalman::StandardBase>
  class OdomMeasurementModel
    : public Kalman::LinearizedMeasurementModel<State<T>, OdomMeasurement<T>, CovarianceBase> {
  public:
    //! State type shortcut definition
    typedef State<T> S;
    
    //! Measurement type shortcut definition
    typedef OdomMeasurement<T> M;
  
    OdomMeasurementModel() {
      // Setup jacobians. As these are static, we can define them once
      // and do not need to update them dynamically

      this->H.setIdentity();
      this->V.setIdentity();
    }
  
    M h(const S& x) const {
      M measurement;
    
      // Robot Frame = something * Lab frame
      // conversion: Lab to robot
      
      measurement.vx() = std::cos(x.theta())*x.vx() + std::sin(x.theta())*x.vy();
      measurement.vy() = -std::sin(x.theta())*x.vx() + std::cos(x.theta())*x.vy();
      measurement.omega() = x.omega();

      return measurement;
    }


    void updateJacobians(const S& x, const double t = 0.05) {

      std::cout << "ODOM Jacobian Updated" << std::endl;

      this->H.setZero();

      this->H(M::OMEGA, S::OMEGA) = 1;
      // Robot Frame = something * Lab frame
      // conversion: Lab to robot
      this->H(M::VX, S::THETA) = -std::sin(x.theta())*x.vx() + std::cos(x.theta())*x.vy();
      this->H(M::VY, S::THETA) = -std::cos(x.theta())*x.vx() - std::sin(x.theta())*x.vy();

      this->H(M::VX, S::VX) = std::cos(x.theta());
      this->H(M::VX, S::VY) = std::sin(x.theta());

      this->H(M::VY, S::VX) = -std::sin(x.theta());
      this->H(M::VY, S::VY) = std::cos(x.theta());
    }

  };
}
