#include "kalman/Matrix.hpp"
#include <kalman/LinearizedSystemModel.hpp>
#include <cmath>
#include <math.h>

// because the variable name is t, I know it's confusing
#define PER t 


#define F32_PI 3.14159265358979f
#define F32_PI_2 1.57079632679489f
#define F32_2_PI 6.28318530717958f

template <typename T>
T angleClamp(T angle)
{
  if (angle > F32_PI)
    {
      angle -= F32_2_PI;
    }
  else if (angle < (-F32_PI))
    {
      angle += F32_2_PI;
    }
  return angle;
}



namespace Robot {
  template <typename T>
  class State : public Kalman::Vector<T, 8> {
  public:
    KALMAN_VECTOR(State, T, 8)


    //! X-position
    static constexpr size_t X = 0;
    //! Y-Position
    static constexpr size_t Y = 1;
    //! Orientation
    static constexpr size_t THETA = 2;
    // Velocities
    static constexpr size_t VX = 3;
    static constexpr size_t VY = 4;
    static constexpr size_t OMEGA = 5;
    // Accelerations
    static constexpr size_t AX = 6;
    static constexpr size_t AY = 7;

    T x() const { return (*this)[X]; }
    T y() const { return (*this)[Y]; }
    T theta() const { return (*this)[THETA]; }
    T vx() const { return (*this)[VX]; }
    T vy() const { return (*this)[VY]; }
    T omega() const { return (*this)[OMEGA]; }
    T ax() const { return (*this)[AX]; }
    T ay() const { return (*this)[AY]; }

    T& x() { return (*this)[X]; }
    T& y() { return (*this)[Y]; }
    T& theta() { return (*this)[THETA]; }
    T& vx() { return (*this)[VX]; }
    T& vy() { return (*this)[VY]; }
    T& omega() { return (*this)[OMEGA]; }
    T& ax() { return (*this)[AX]; }
    T& ay() { return (*this)[AY]; }
  };
}


namespace Robot {

  template <typename T>
  class Twist : public Kalman::Vector<T, 3>
  {
  public:

    KALMAN_VECTOR(Twist, T, 3)
    // Velocities
    static constexpr size_t RVX = 0;
    static constexpr size_t RVY = 1;
    static constexpr size_t ROMEGA = 2;

    T rvx() const { return (*this)[RVX]; }
    T rvy() const { return (*this)[RVY]; }
    T romega() const { return (*this)[ROMEGA]; }

    T& rvx() { return (*this)[RVX]; }
    T& rvy() { return (*this)[RVY]; }
    T& romega() { return (*this)[ROMEGA]; }
  };



  template <typename T,
	    template <class> class CovarianceBase = Kalman::StandardBase>
  class SystemModel : public Kalman::LinearizedSystemModel<State<T>, Twist<T>, CovarianceBase> {
  public:

    typedef State<T> S ;
    typedef Twist<T> C ;
    T diff_;

    // Non Linear Functtion, refer the PDF for this

    void set_time_diff(T diff) {
      diff_ = diff;
    }


    S f(const S& x, const C& u, const double t = 0.05) const {
      S x_;

      x_.x()  = x.x() + x.vx()*t + 0.5*(t*t)*x.ax();
      x_.y()  = x.y() + x.vy()*t + x.ay()*(t*t)*0.5;

      x_.theta()  = angleClamp(x.theta() + x.omega()*t); // Clamping the angle, hope it won't affect the jacobian

      // x_.vx() = (cos(x.theta()) * u.rvx() - sin(x.theta()) * u.rvy()) + x.ax()*t;
      // x_.vy() = (sin(x.theta()) * u.rvx() + cos(x.theta()) * u.rvy()) + x.ay()*t;
      //      x_.omega()  = u.romega();

      x_.vx() = x.vx() + x.ax()*t;
      x_.vy() = x.vy() + x.ay()*t;
      x_.omega()  = x.omega();

      x_.ax() = x.ax();
      x_.ay() = x.ay();

      return x_;
    }

protected:
    // Update the Jecobian
    void updateJacobians(const S &x, const C &u, const double t = 0.05) {
      

      // F = df/dx (Jacobian of state transition w.r.t. the state)
      this->F.setZero();

      std::cout << "SYSTEM JACOBIAN UPDATED" << std::endl;


      // First Set the ones that are 1
      this->F(S::X, S::X) = 1;
      this->F(S::Y, S::Y) = 1;
      this->F(S::THETA, S::THETA) = 1;
      this->F(S::AX, S::AX) = 1;
      this->F(S::AY, S::AY) = 1;
      this->F(S::OMEGA, S::OMEGA) = 1;
      this->F(S::VX, S::VX) = 1;
      this->F(S::VY, S::VY) = 1;
     
      // double this1 = (-sin(x.theta())*u.rvx() - cos(x.theta())*u.rvy());
      // double this2 = (cos(x.theta())*u.rvx() - sin(x.theta())*u.rvy());


      // // This part changed in the new one 
      // this->F(S::X, S::THETA) = 0;
      // this->F(S::Y, S::THETA) = 0;



      // // use things
      // this->F(S::VX, S::THETA) = 0;
      // this->F(S::VY, S::THETA) = 0;



      // use time period
      this->F(S::X, S::VX)        = t;
      this->F(S::Y, S::VY)        = t;
      this->F(S::THETA, S::OMEGA) = t;

      // this->F(S::OMEGA, S::THETA) = t;

      this->F(S::VX, S::AX)       = t;
      this->F(S::VY, S::AY)       = t;

      // use 0.5T^2

      this->F(S::X, S::AX) = 0.5*t*t;
      this->F(S::Y, S::AY) = 0.5*t*t;

      std::cout << "JACCC" << std::endl;

      std::cout << this->F << std::endl;

      std::cout << "JACCC END" << std::endl;

      // W = df/dw (Jacobian of state transition w.r.t. the noise)
      // this->W.setIdentity();

      this->W.setIdentity();


      // this->W(S::VX, C::RVX) = std::cos(x.theta());
      // this->W(S::VX, C::RVY) = -std::sin(x.theta());

      // this->W(S::VY, C::RVX) = std::sin(x.theta());
      // this->W(S::VY, C::RVX) = std::cos(x.theta());

      // W Matrix Has been set


    }
  };
}
