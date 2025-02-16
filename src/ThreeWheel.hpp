#include "kalman/Matrix.hpp"
#include "kalman/StandardBase.hpp"
#include "wheel.hpp"
#include <cstddef>

#include "kalman/LinearizedMeasurementModel.hpp"

namespace Robot {
  

  template <typename T> class WheelMeasurement: public Kalman::Vector<T, 3> {
  public:
    KALMAN_VECTOR(WheelMeasurement, T, 3)

    static constexpr std::size_t OMEGA_R = 0;
    static constexpr std::size_t OMEGA_L = 1;
    static constexpr std::size_t OMEGA_M = 2;

    T omega_r() const { return (*this)[OMEGA_R]; }
    T &omega_r() { return (*this)[OMEGA_R]; }

    T omega_l() const { return (*this)[OMEGA_L]; }
    T &omega_l() { return (*this)[OMEGA_L]; }

    T omega_m() const { return (*this)[OMEGA_M]; }
    T &omega_m() { return (*this)[OMEGA_M]; }

  };

  template <typename T,
	    template <class> class CovarianceBase = Kalman::StandardBase>
  class WheelMeasurementModel 
    : public Kalman::LinearizedMeasurementModel<State<T>, WheelMeasurement<T>, CovarianceBase> {
  public:
    //! State type shortcut definition
    typedef State<T> S;
    
    //! Measurement type shortcut definition
    typedef WheelMeasurement<T> M;

    // T left_radius, right_radius, mid_radius, wheel_radius;
    mutable Encoder<T> left, right, mid;
  
    WheelMeasurementModel(T left_x, T left_y, T left_alpha, T right_x, T right_y, T right_alpha, T mid_x, T mid_y, T mid_alpha, T wheel_radius_):
      left(left_x, left_y, left_alpha, wheel_radius_),
      right(right_x, right_y, right_alpha, wheel_radius_),
      mid(mid_x, mid_y, mid_alpha, wheel_radius_)
    {

      //      std::cout << "left_rad : " << left_radius << " rignt_rad: " << right_radius << "mid_radius: " << mid_radius << "wheel_rad " << wheel_radius << std::endl;
      // Setup jacobians. As these are static, we can define them once
      // and do not need to update them dynamically



      this->H.setIdentity();
      this->V.setIdentity();
    }
  
    M h(const S& x) const {
      M measurement;
      measurement.omega_l() = left.calculate(x.theta(), x.vx(), x.vy(), x.omega()).omega;
      measurement.omega_r() = right.calculate(x.theta(), x.vx(), x.vy(), x.omega()).omega;
      measurement.omega_m() = mid.calculate(x.theta(), x.vx(), x.vy(), x.omega()).omega;

      std::cout << "Measurement Is " << std::endl << measurement << std::endl;

      return measurement;
    }


    void updateJacobians(const S& x, const double t = 0.05) {

      std::cout << "Wheel Jacobian Updated" << std::endl;

      this->H.setIdentity();

      this->H(M::OMEGA_L, S::X) = 0;
      this->H(M::OMEGA_L, S::Y) = 0;
      this->H(M::OMEGA_L, S::THETA) = left.calculate(x.theta(), x.vx(), x.vy(), x.omega()).dtheta;
      this->H(M::OMEGA_L, S::VX) = left.calculate(x.theta(), x.vx(), x.vy(), x.omega()).dvx;
      this->H(M::OMEGA_L, S::VY) = left.calculate(x.theta(), x.vx(), x.vy(), x.omega()).dvy;
      this->H(M::OMEGA_L, S::OMEGA) = left.calculate(x.theta(), x.vx(), x.vy(), x.omega()).domega;
      this->H(M::OMEGA_L, S::AX) = 0;
      this->H(M::OMEGA_L, S::AY) = 0;


      this->H(M::OMEGA_R, S::X) = 0;
      this->H(M::OMEGA_R, S::Y) = 0;
      this->H(M::OMEGA_R, S::THETA) = right.calculate(x.theta(), x.vx(), x.vy(), x.omega()).dtheta;
      this->H(M::OMEGA_R, S::VX) = right.calculate(x.theta(), x.vx(), x.vy(), x.omega()).dvx;
      this->H(M::OMEGA_R, S::VY) = right.calculate(x.theta(), x.vx(), x.vy(), x.omega()).dvy;
      this->H(M::OMEGA_R, S::OMEGA) = right.calculate(x.theta(), x.vx(), x.vy(), x.omega()).domega;
      this->H(M::OMEGA_R, S::AX) = 0;
      this->H(M::OMEGA_R, S::AY) = 0;


      this->H(M::OMEGA_M, S::X) = 0;
      this->H(M::OMEGA_M, S::Y) = 0;
      this->H(M::OMEGA_M, S::THETA) = mid.calculate(x.theta(), x.vx(), x.vy(), x.omega()).dtheta;
      this->H(M::OMEGA_M, S::VX) = mid.calculate(x.theta(), x.vx(), x.vy(), x.omega()).dvx;
      this->H(M::OMEGA_M, S::VY) = mid.calculate(x.theta(), x.vx(), x.vy(), x.omega()).dvy;
      this->H(M::OMEGA_M, S::OMEGA) = mid.calculate(x.theta(), x.vx(), x.vy(), x.omega()).domega;
      this->H(M::OMEGA_M, S::AX) = 0;
      this->H(M::OMEGA_M, S::AY) = 0;


      std::cout << "Wheel Jacobian Updated" << std::endl;
      std::cout << "Wheel H is" << std::endl;
      std::cout << this->H << std::endl;
      std::cout << std::endl << std::endl;
    }

  };
}
