#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /*
  * Initialize PID controller with the coefficients as the input values.
  */
  
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;

  d_error = 0;
  p_error = 0;
  i_error = 0;
  
}

void PID::UpdateError(double cte) {
  /**
  * Update PID errors based on cte.
  */

  // d_error is difference from old cte (p_error) to the new cte
  d_error = (cte - p_error);

  // p_error gets set to the new cte
  p_error = cte;

  // i_error is the sum of ctes to this point
  i_error += cte;
  
}

double PID::TotalError() {
  /**
  * Calculate and return the total error which is the 
  * new steering angle value
  */
  double pid_output = -Kp * p_error - Kd * d_error - Ki * i_error;
  if (pid_output > 1.0){
    pid_output = 1.0;
  }
  if (pid_output < -1.0){
    pid_output = -1.0;
  }

  return pid_output;
}

double PID::get_p_error() {
  return p_error;
}

double PID::get_d_error() {
  return d_error;
}

double PID::get_i_error() {
  return i_error;
}