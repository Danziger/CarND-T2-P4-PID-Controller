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
  double kp_;
  double ki_;
  double kd_;

  double cte_prev_ = 0;
  double cte_int_ = 0;

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
  void init(const double Kp, const double Ki, const double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  double update(const double cte, const double min, const double max);

  /*
  * Calculate the total PID error.
  */
  double getTotalError();
};

#endif /* PID_H */
