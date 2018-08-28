#include "PID.h"
#include <math.h>

using namespace std;

/*
 * PID class implementation
 */

PID::PID() : Kp(0.0), Ki(0.0), Kd(0.0) {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
}

void PID::UpdateError(double cte) {
  d_error = cte - p_error;  // p_error contains the previous cte
  p_error = cte;
  i_error += cte;
}

double PID::TotalError() { return -Kp * p_error - Ki * i_error - Kd * d_error; }
