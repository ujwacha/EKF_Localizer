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
  class State : public Kalman::Vector<T, 9> {
  public:
    KALMAN_VECTOR(State, T, 9)

    // some comment on the whole state
    // x y and theta are on global frame
    // the rest are on robot frame

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
    static constexpr size_t YAW_BIAS = 8;

    T x() const { return (*this)[X]; }
    T y() const { return (*this)[Y]; }
    T theta() const { return (*this)[THETA]; }
    T vx() const { return (*this)[VX]; }
    T vy() const { return (*this)[VY]; }
    T omega() const { return (*this)[OMEGA]; }
    T ax() const { return (*this)[AX]; }
    T ay() const { return (*this)[AY]; }
    T yaw_bias() const { return (*this)[YAW_BIAS]; }

    T& x() { return (*this)[X]; }
    T& y() { return (*this)[Y]; }
    T& theta() { return (*this)[THETA]; }
    T& vx() { return (*this)[VX]; }
    T& vy() { return (*this)[VY]; }
    T& omega() { return (*this)[OMEGA]; }
    T& ax() { return (*this)[AX]; }
    T& ay() { return (*this)[AY]; }
    T& yaw_bias() { return (*this)[YAW_BIAS]; }
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

      x_.x() = x.x()
	+ rotation_upper_x(x.theta(), x.vx(), x.vy())*t
	+ rotation_upper_x(x.theta(), x.ax(), x.ay())*t*t*0.5;

      x_.y() = x.y()
	+ rotation_lower_y(x.theta(), x.vx(), x.vy())*t
	+ rotation_lower_y(x.theta(), x.ax(), x.ay())*t*t*0.5;

      x_.theta() = angleClamp(x.theta() + t*x.omega());

      x_.vx() = u.rvx();

      x_.vy() = u.rvy();

      x_.omega() = u.romega();

      x_.ax() = x.ax();

      x_.ay() = x.ay();

      x_.yaw_bias() = x.yaw_bias();

      return x_;
    }

    inline T rotation_upper_x(T theta, T front, T back) const {
      theta = angleClamp(theta);
      return std::cos(theta)*front - std::sin(theta)*back;
    }

    inline T rotation_lower_y(T theta, T front, T back) const {
      theta = angleClamp(theta);
      return std::sin(theta)*front + std::cos(theta)*back;
    }

    inline T rotation_upper_dx(T theta, T front, T back) const {
      theta = angleClamp(theta);
      return -std::sin(theta)*front - std::cos(theta)*back;
    }

    inline T rotation_lower_dy(T theta, T front, T back) const {
      theta = angleClamp(theta);
      return std::cos(theta)*front - std::sin(theta)*back;
    }


protected:
    // Update the Jecobian
    void updateJacobians(const S &x, const C &u, const double t = 0.05) {
      // F = df/dx (Jacobian of state transition w.r.t. the state)
      this->F.setZero();


      T theta = angleClamp(x.theta());
      // for x
      this->F(S::X, S::X) = 1;
      // for y
      this->F(S::Y, S::Y) = 1;

      // for theta
      this->F(S::X, S::THETA) =
	t * rotation_upper_dx(theta, x.vx(), x.vy())
	+ 0.5*t*t * rotation_upper_dx(theta, x.ax(), x.ay());

      this->F(S::Y, S::THETA) = 
	t * rotation_lower_dy(theta, x.vx(), x.vy())
	+ 0.5*t*t * rotation_lower_dy(theta, x.ax(), x.ay());

      this->F(S::THETA, S::THETA) = 1;

      // for vx
      this->F(S::X, S::VX) = t*std::cos(theta);
      this->F(S::Y, S::VX) = t*std::sin(theta);

      // for vy
      this->F(S::X, S::VY) = -t*std::sin(theta);
      this->F(S::Y, S::VY) = t*std::cos(theta);

      // for omega
      this->F(S::THETA, S::OMEGA) = t;

      // for ax
      this->F(S::X, S::AX) = 0.5*t*t*std::cos(theta);
      this->F(S::Y, S::AX) = 0.5*t*t*std::sin(theta);
      this->F(S::AX, S::AX) = 1;

      // for ay
      this->F(S::X, S::AY) = -0.5*t*t*std::sin(theta);
      this->F(S::Y, S::AY) = 0.5*t*t*std::cos(theta);
      this->F(S::AY, S::AY) = 1;


      this->F(S::YAW_BIAS, S::YAW_BIAS) = 1;
      
      this->W.setIdentity();

      // literally the same thing lol
      // this->W.setZero();
      // this->W(S::VX, C::RVX) = 1;
      // this->W(S::VY, C::RVY) = 1;
      // this->W(S::OMEGA, C::ROMEGA) = 1;

      // std::cout << "W = " << std::endl;
      // std::cout << this->W << std::endl;
      // char c;
      // std::cin >> c;
      // W Matrix Has been set

    }
  };
}
