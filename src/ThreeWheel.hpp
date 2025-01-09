#include "kalman/Matrix.hpp"
#include "kalman/StandardBase.hpp"
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

    T left_radius, right_radius, mid_radius, wheel_radius;
  
    WheelMeasurementModel(T left_radius_, T right_radius_, T mid_radius_, T wheel_radius_):
      left_radius(left_radius_),
      right_radius(right_radius_),
      mid_radius(mid_radius_),
      wheel_radius(wheel_radius_)
    {

      std::cout << "left_rad : " << left_radius << " rignt_rad: " << right_radius << "mid_radius: " << mid_radius << "wheel_rad " << wheel_radius << std::endl;
      // Setup jacobians. As these are static, we can define them once
      // and do not need to update them dynamically



      this->H.setIdentity();
      this->V.setIdentity();
    }
  
    M h(const S& x) const {
      M measurement;

      T vxr = std::cos(x.theta())*x.vx() + std::sin(x.theta())*x.vy();
      T vyr = -std::sin(x.theta())*x.vx() + std::cos(x.theta())*x.vy();

      measurement.omega_l() = vxr - this->left_radius*x.omega();
      measurement.omega_r() = vxr + this->right_radius*x.omega();
      measurement.omega_m() = vyr - this->mid_radius*x.omega();

      measurement.omega_l() /= this->wheel_radius;
      measurement.omega_r() /= this->wheel_radius;
      measurement.omega_m() /= this->wheel_radius;


      return measurement;
    }


    void updateJacobians(const S& x, const double t = 0.05) {

      std::cout << "Wheel Jacobian Updated" << std::endl;

      this->H.setIdentity();

      this->H(M::OMEGA_L, S::X) = 0;
      this->H(M::OMEGA_L, S::Y) = 0;
      this->H(M::OMEGA_L, S::THETA) = (-std::sin(x.theta())*x.vx() + std::cos(x.theta())*x.vy()) / this->wheel_radius;
      this->H(M::OMEGA_L, S::VX) = std::cos(x.theta()) / this->wheel_radius;
      this->H(M::OMEGA_L, S::VY) = std::sin(x.theta()) / this->wheel_radius;
      this->H(M::OMEGA_L, S::OMEGA) = -this->left_radius /  this->wheel_radius;
      this->H(M::OMEGA_L, S::AX) = 0;
      this->H(M::OMEGA_L, S::AY) = 0;


      this->H(M::OMEGA_R, S::X) = 0;
      this->H(M::OMEGA_R, S::Y) = 0;
      this->H(M::OMEGA_R, S::THETA) = (-std::sin(x.theta())*x.vx() + std::cos(x.theta())*x.vy()) / this->wheel_radius;
      this->H(M::OMEGA_R, S::VX) = std::cos(x.theta()) / this->wheel_radius;
      this->H(M::OMEGA_R, S::VY) = std::sin(x.theta()) / this->wheel_radius;
      this->H(M::OMEGA_R, S::OMEGA) = this->right_radius /  this->wheel_radius;
      this->H(M::OMEGA_R, S::AX) = 0;
      this->H(M::OMEGA_R, S::AY) = 0;


      this->H(M::OMEGA_M, S::X) = 0;
      this->H(M::OMEGA_M, S::Y) = 0;
      this->H(M::OMEGA_M, S::THETA) = (-std::cos(x.theta())*x.vx() - std::sin(x.theta())*x.vy()) / this->wheel_radius;
      this->H(M::OMEGA_M, S::VX) = -std::sin(x.theta()) / this->wheel_radius;
      this->H(M::OMEGA_M, S::VY) = std::cos(x.theta()) / this->wheel_radius;
      this->H(M::OMEGA_M, S::OMEGA) = -this->mid_radius /  this->wheel_radius;
      this->H(M::OMEGA_M, S::AX) = 0;
      this->H(M::OMEGA_M, S::AY) = 0;


      std::cout << "Wheel Jacobian Updated" << std::endl;
      std::cout << "Wheel H is" << std::endl;
      std::cout << this->H << std::endl;
      std::cout << std::endl << std::endl;


    }

  };
}
