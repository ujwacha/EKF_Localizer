#include "kalman/Matrix.hpp"
#include "kalman/StandardBase.hpp"
#include <cstddef>

#include "kalman/LinearizedMeasurementModel.hpp"

#define PI 3.1415

namespace Robot {
  

  template <typename T> class TFMiniMeasurement: public Kalman::Vector<T, 6> {
  public:
    KALMAN_VECTOR(TFMiniMeasurement, T, 3)

    static constexpr std::size_t D1 = 0;
    static constexpr std::size_t D2 = 1;
    static constexpr std::size_t D3 = 2;

    T d1() const { return (*this)[D1]; }
    T &d1() { return (*this)[D1]; }

    T d2() const { return (*this)[D2]; }
    T &d2() { return (*this)[D2]; }

    T d3() const { return (*this)[D3]; }
    T &d3() { return (*this)[D3]; }



  };

  template <typename T,
	    template <class> class CovarianceBase = Kalman::StandardBase>
  class TFMiniMeasurementModel 
    : public Kalman::LinearizedMeasurementModel<State<T>, TFMiniMeasurement<T>, CovarianceBase> {
  public:
    //! State type shortcut definition
    typedef State<T> S;
    
    //! Measurement type shortcut definition
    typedef OdomMeasurement<T> M;

    T angle1, angle2, angle3, l, b;

    enum State {
      L1,
      L2,
      B1,
      B2,
    };
   

    State current_state(const S& x, T angle = 0.0) {
      T alpha, beta, theta, sigma;

      theta = std::arctan(x.x()/x.y());
      alpha = std::arctan(x.y()/(b - x.x()));
      beta = std::arctan((l-x.y())/(b - x.x()));
      sigma = std::arctan((l-x.y())/x.x());


      T th = x.theta() + angle;

      if (th >= -alpha && th <= beta) return State::L2;
      if (th >= beta && th <= PI - sigma) return State::B2;
      if (th <= -alpha && th >= -(PI - theta)) return State::B1;
      if ((th >= sigma && th <= PI) || (th >= -PI && th <= -theta)) return State::L1;
      // If everythin fails
      return State::L1;

    }

    T get_d(const S& x, T angle) {
      T d;
      switch (current_state(x, angle)) {
      case State::L1:
	d = (-x.x())/std::cos(x.theta());
	break;
      case State::L2:
	d = (b - x.x())/std::cos(x.theta());
	break;
      case State::B1:
	d = (-x.y())/std::sin(x.theta());
	break;
      case State::B2:
	d = (l - x.y())/std::sin(x.theta());
      }

      return d;
    }

  
    OdomMeasurementModel(T angle1_, T angle2_, T angle3_, T l_, T b_):
      angle1(angle1_), angle2(angle2_), angle3(angle3_), l(l_), b(b_)
    {
      this->H.setIdentity();
      this->V.setIdentity();

      // this->V.setZero();
      // this->V(M::X, M::X) = 1;
      // this->V(M::Y, M::Y) = 1;
      // this->V(M::THETA, M::THETA) = 1;
      // this->V(M::VX, M::VX) = 3;
      // this->V(M::VY, M::VY) = 3;
      // this->V(M::OMEGA, M::OMEGA) = 5;
    }
  
    M h(const S& x) const {
      M measurement;

      m.d1() = get_d(x, angle1);
      m.d2() = get_d(x, angle2);
      m.d3() = get_d(x, angle3);
  
      return measurement;
    }

    void updateJacobians( const S& x )
    {
      // H = dh/dx (Jacobian of measurement function w.r.t. the state)
      this->H.setZero();
        
      // Robot position as (x,y)-vector
      // This uses the Eigen template method to get the first 2 elements of the vector
      Kalman::Vector<T, 2> position = x.template head<2>();
        
      // Distance of robot to landmark 1
      Kalman::Vector<T, 2> delta1 = position - landmark1;
        
      // Distance of robot to landmark 2
      Kalman::Vector<T, 2> delta2 = position - landmark2;
        
      // Distances
      T d1 = std::sqrt( delta1.dot(delta1) );
      T d2 = std::sqrt( delta2.dot(delta2) );
        
      // partial derivative of meas.d1() w.r.t. x.x()
      this->H( M::D1, S::X ) = delta1[0] / d1;
      // partial derivative of meas.d1() w.r.t. x.y()
      this->H( M::D1, S::Y ) = delta1[1] / d1;
        
      // partial derivative of meas.d1() w.r.t. x.x()
      this->H( M::D2, S::X ) = delta2[0] / d2;
      // partial derivative of meas.d1() w.r.t. x.y()
      this->H( M::D2, S::Y ) = delta2[1] / d2;
    }
  
  
  };
}
