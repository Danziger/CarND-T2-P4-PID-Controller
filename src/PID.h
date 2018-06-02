#ifndef PID_H
#define PID_H



class PID {
public:
  /*
  * Errors
  */
  double err_p_ = 0;
  double err_i_ = 0;
  double err_d_ = 0;
  double err_total_ = 0;

  /*
  * Coefficients
  */ 
  double Kp_;
  double Ki_;
  double Kd_;

  double Ap_;
  double Ai_;
  double Ad_;

  double Wi_;

  double pv_prev_ = 0;
  double pv_int_ = 0;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void init(const double Kp, const double Lp, const double Ki, const double Li, const double Kd, const double Ld, double const Wi);

  /*
  * Update the PID error variables given cross track error.
  */
  double update(const double cte, const double speed, const double min, const double max);

  /*
  * Calculate the total PID error.
  */
  double getTotalError();
};

#endif /* PID_H */
