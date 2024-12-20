#include "kalman/Matrix.hpp"
#include "kalman/StandardBase.hpp"
#include <cstddef>

#include "kalman/LinearizedMeasurementModel.hpp"

namespace Robot {
  

  template <typename T> class OdomMeasurement : public Kalman::Vector<T, 6> {
  public:
    KALMAN_VECTOR(OdomMeasurement, T, 6)

    static constexpr std::size_t X = 0;
    static constexpr std::size_t Y = 1;
    static constexpr std::size_t THETA = 2;
    static constexpr std::size_t VX = 3;
    static constexpr std::size_t VY = 4;
    static constexpr std::size_t OMEGA = 5;

    T x() const { return (*this)[X]; }
    T &x() { return (*this)[X]; }

    T y() const { return (*this)[Y]; }
    T &y() { return (*this)[Y]; }

    T theta() const { return (*this)[THETA]; }
    T &theta() { return (*this)[THETA]; }

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
      //this->H.setIdentity();

      this->H(M::X, S::X) = 1;
      this->H(M::Y, S::Y) = 1;
      this->H(M::THETA, S::THETA) = 1;
      this->H(M::VX, S::VX) = 1;
      this->H(M::VY, S::VY) = 1;
      this->H(M::OMEGA, S::OMEGA) = 1;

      //  this->V.setIdentity();

      this->V(M::X, M::X) = 0.1;
      this->V(M::Y, M::Y) = 0.1;
      this->V(M::THETA, M::THETA) = 0.1;
      this->V(M::VX, M::VX) = 0.1;
      this->V(M::VY, M::VY) = 0.1;
      this->V(M::OMEGA, M::OMEGA) = 0.1;
    }
  
    M h(const S& x) const {
      M measurement;
    
      measurement.x() = x.x();
      measurement.y() = x.y();
      measurement.theta() = x.theta();
      measurement.vx() = x.vx();
      measurement.vy() = x.vy();
      measurement.omega() = x.omega();
    
      return measurement;
    }

  
  
  };
}
