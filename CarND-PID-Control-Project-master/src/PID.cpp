#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  // P: is the amount to steer in proportion to the x-track error, the larger the error, 
  //    the faster the turn rate, this is calculated via -P * CTE for the turn rate
  // I: is the bias adjustment, turn value is calculated via I * (sum of CTE over time)
  // D: is the some constant which is the amount of counter-steer to apply, 
  //    turn value is calculated via D * (CTE_t - CTE_{t-1})
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  sum_cte = 0;
  prev_cte = 0;
}


void PID::UpdateError(double cte) {
	sum_cte += cte;
    p_error = - Kp * cte;
    i_error = - Ki * sum_cte;
    d_error = - Kd * (cte - prev_cte);
    prev_cte = cte;
}

double PID::TotalError() {
	return p_error + i_error + d_error;
}

