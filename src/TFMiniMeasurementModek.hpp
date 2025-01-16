#include "kalman/Matrix.hpp"
#include "kalman/StandardBase.hpp"
#include <cstddef>

#include "kalman/LinearizedMeasurementModel.hpp"
#include "Distance.hpp"

#define PI 3.1415

namespace Robot {
  template <typename T> class TFMiniMeasurement: public Kalman::Vector<T, 3> {
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
    typedef TFMiniMeasurement<T> M;

    mutable Distance<T> d1, d2, d3;

 
    TFMiniMeasurementModel(T angle1_, T r1_, T angle2_, T r2_, T angle3_, T r3_, T l_, T b_):
      d1(angle1_, r1_, l_, b_),
      d2(angle2_, r2_, l_, b_),
      d3(angle3_, r3_, l_, b_)
    {
      this->H.setIdentity();
      this->V.setIdentity();

    }
  
    M h(const S& x) const {
      M measurement;

      measurement.d1() = d1.get_distance(x.x(), x.y(), x.theta());
      measurement.d2() = d2.get_distance(x.x(), x.y(), x.theta());
      measurement.d3() = d3.get_distance(x.x(), x.y(), x.theta());

      return measurement;
    }

    void updateJacobians( const S& x )
    {
      this->H.setZero();
      this->H(M::D1, S::X) = d1.get_dx();
      this->H(M::D1, S::Y) = d1.get_dy();
      this->H(M::D1, S::THETA) = d1.get_dth();

      this->H(M::D2, S::X) = d2.get_dx();
      this->H(M::D2, S::Y) = d2.get_dy();
      this->H(M::D2, S::THETA) = d2.get_dth();

      this->H(M::D3, S::X) = d3.get_dx();
      this->H(M::D3, S::Y) = d3.get_dy();
      this->H(M::D3, S::THETA) = d3.get_dth();
    }
  
  };
}
