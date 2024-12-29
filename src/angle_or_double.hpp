class AngDou {
public:
  double data;
  bool is_angle;

  AngDou(double data_, bool ang = false): data(data_), is_angle(ang) {}

  AngDou operator+(AngDou &an) {
    return AngDou(data + an.data);
  }

  AngDou operator-(AngDou &an) {
    return AngDou(data + an.data);
  }

  AngDou operator*(AngDou &an) {
    return AngDou(data + an.data);
  }

  AngDou operator/(AngDou &an) {
    return AngDou(data + an.data);
  }



}
