#include "kalman/LinearizedMeasurementModel.hpp"
#include "kalman/Matrix.hpp"

namespace Robot {
  

  template <typename T> class ImuMeasurement : public Kalman::Vector<T, 3> {
  public:
    KALMAN_VECTOR(ImuMeasurement, T, 3)

    static constexpr size_t AX = 0;
    static constexpr size_t AY = 1;
    static constexpr size_t YAW = 2;

    T ax() const { return (*this)[AX]; }
    T ay() const { return (*this)[AY]; }
    T yaw() const { return (*this)[YAW]; }

    T &ax() { return (*this)[AX]; }
    T &ay() { return (*this)[AY]; }
    T &yaw() { return (*this)[YAW]; }
  };

  template <typename T,
	    template <class> class CovarianceBase = Kalman::StandardBase>
  class ImuMeasurementModel
    : public Kalman::LinearizedMeasurementModel<
    State<T>, ImuMeasurement<T>, CovarianceBase> {

  public:

    typedef  State<T> S;
    
    //! Measurement type shortcut definition
    typedef ImuMeasurement<T> M;
  
    ImuMeasurementModel() {
      this->H.setZero();

      this->H(M::AX, S::AX) = 1;
      this->H(M::AY, S::AY) = 1;
      this->H(M::YAW, S::THETA) = 1;

      // Seti The Noise Covariance
      // this->V.set();

      this->V.setIdentity();
      //      this->V(M::YAW, M::YAW) = 0.01;
    }

    M h(const S& x) const {
      M measurement;
      measurement.ax() = std::cos(x.theta())*x.ax() + std::sin(x.theta())*x.ay();
      measurement.ay() = -std::sin(x.theta())*x.ax() + std::cos(x.theta())*x.ay();
      measurement.yaw() = x.theta();

      return measurement;
    }

    void updateJacobians(const S& x, const double t = 0.05) {
      this->H.setZero();

      this->H(M::AX, S::THETA) = -std::sin(x.theta())*x.ax() + std::cos(x.theta())*x.ay();
      this->H(M::AY, S::THETA) = -std::cos(x.theta())*x.ax() - std::sin(x.theta())*x.ay();
      this->H(M::YAW, S::THETA) = 1;

      // Row
      this->H(M::AX, S::AX) = std::cos(x.theta());
      this->H(M::AX, S::AY) = std::sin(x.theta());

      //Row
      this->H(M::AY, S::AX) = -std::sin(x.theta());
      this->H(M::AY, S::AY) = std::cos(x.theta());


      // std::cout << "IMU Jacobian Updated" << std::endl;
      // std::cout << "IMU H is" << std::endl;
      // std::cout << this->H << std::endl;
      // std::cout << std::endl << std::endl;
    }
 
  };
}
