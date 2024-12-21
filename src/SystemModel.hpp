#include "kalman/Matrix.hpp"
#include <kalman/LinearizedSystemModel.hpp>
#include <cmath>
#include <math.h>

#define PER 0.05f


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


    S f(const S& x, const C& u) const {
      S x_;

      x_.x()  = x.x() + (cos(x.theta()) * u.rvx() - sin(x.theta()) * u.rvy())*PER + 0.5*(PER * PER)*x.ax();
      x_.y()  = x.y() + (sin(x.theta()) * u.rvx() + cos(x.theta()) * u.rvy())*PER + x.ay()*(PER*PER)*0.5f;
      x_.theta()  = angleClamp(x.theta() + x.omega()*PER); // Clamping the angle, hope it won't affect the jacobian

      x_.vx() = (cos(x.theta()) * u.rvx() - sin(x.theta()) * u.rvy()) + x.ax()*PER;
      x_.vy() = (sin(x.theta()) * u.rvx() + cos(x.theta()) * u.rvy()) + x.ay()*PER;
      x_.omega()  = u.romega();

      x_.ax() = x.ax();
      x_.ay() = x.ay();

      return x_;
    }

protected:
    // Update the Jecobian
    void updateJacobians(const S &x, const C &u) {
      // F = df/dx (Jacobian of state transition w.r.t. the state)
      this->F.setZero();



      // First Set the ones that are 1
      this->F(S::X, S::X) = 1;
      this->F(S::Y, S::Y) = 1;
      this->F(S::THETA, S::THETA) = 1;
      this->F(S::AX, S::AX) = 1;
      this->F(S::AY, S::AY) = 1;
     
      float this1 = (-sin(x.theta())*u.rvx() - cos(x.theta())*u.rvy());
      float this2 = (cos(x.theta())*u.rvx() - sin(x.theta())*u.rvy());

      // use things
      this->F(S::X, S::THETA) = PER*this1;
      this->F(S::Y, S::THETA) = PER*this2;
      this->F(S::VX, S::THETA) = this1;
      this->F(S::VY, S::THETA) = this2;

      // use time period

      this->F(S::THETA, S::OMEGA) = PER;
      this->F(S::VX, S::AX) = PER;
      this->F(S::VY, S::AY) = PER;

      // use 0.5T^2

      this->F(S::X, S::AX) = 0.5*PER*PER;
      this->F(S::Y, S::AY) = 0.5*PER*PER;


      // W = df/dw (Jacobian of state transition w.r.t. the noise)
      this->W.setIdentity();
      // TODO: more sophisticated noise modelling
      //       i.e. The noise affects the the direction in which we move as
      //       well as the velocity (i.e. the distance we move)
    }
  };
}
