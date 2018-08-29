#include "PID.h"
#include <math.h>

using namespace std;

/*
 * PID class implementation
 */

PID::PID() : p_error(0.0), i_error(0.0), d_error(0.0), Kp(0.0), Ki(0.0), Kd(0.0), initialized(false) {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
}

void PID::UpdateError(double cte) {
  if (!initialized) {
    // Sets initial p error
    p_error = cte;
    initialized = true;
  }
  d_error = cte - p_error;  // p_error contains the previous cte
  p_error = cte;
  i_error += cte;
}

double PID::TotalError() { return -Kp * p_error - Ki * i_error - Kd * d_error; }
