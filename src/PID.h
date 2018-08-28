#ifndef PID_H
#define PID_H

class PID {
 public:
  /*
   * Default Constructor, sets all the coefficients to zero
   */
  PID();

  /*
   * Destructor.
   */
  virtual ~PID();

  /*
   * Initialize PID with the given coefficient values.
   *
   * @param Kp The proportional error coefficient
   * @param Ki The integral error coefficient
   * @param Kd The derivative error coefficient
   */
  void Init(double Kp, double Ki, double Kd);

  /*
   * Update the PID error variables given cross track error.
   *
   * @param cte Cross track error value
   */
  void UpdateError(double cte);

  /*
   * Calculate the total PID error for this iteration.
   */
  double TotalError();

 private:
  /*
   * Errors for proportional, integral and derivative values
   */
  double p_error;
  double i_error;
  double d_error;

  /*
   * Coefficients for proportional, integral and derivative errors
   */
  double Kp;
  double Ki;
  double Kd;
};

#endif /* PID_H */
