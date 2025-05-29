class Controller {
public:
  struct controller_twist {
    double vx;
    double vy;
    double w;
  };

  double x1, y1, x2, y2, x3, y3;
  double a1, b1, a2, b2, a3, b3;
  double r;
  Eigen::Matrix3d mat;


  Controller() = default;

  Controller(
	     double _x1, double _y1, double theta_1,
	     double _x2, double _y2,double theta_2,
	     double _x3, double _y3, double theta_3,
	     double _r
	     )
    :
    x1(_x1), y1(_y1),
    x2(_x2), y2(_y2),
    x3(_x3), y3(_y3),
    a1(std::cos(theta_1)), b1(std::sin(theta_1)),
    a2(std::cos(theta_2)), b2(std::sin(theta_2)),
    a3(std::cos(theta_3)), b3(std::sin(theta_3)),
    r(_r)
  {

    mat << a1/r, b1/r, (b1*x1 - a1*y1)/r,
      a2/r, b2/r, (b2*x2 - a2*y2)/r,
      a3/r, b3/r, (b3*x3 - a3*y3)/r ;

    mat = mat.inverse().eval();
  }

  controller_twist
  get_output(double omega_1, double omega_2, double omega_3) {
    Eigen::Vector3d vec ;
    vec << omega_1, omega_2, omega_3;

    Eigen::Vector3d out = mat * vec;

    controller_twist t;
    t.vx = out(0);
    t.vy = out(1);
    t.w = out(2);

    return t;
  }
};
