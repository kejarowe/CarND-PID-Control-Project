#include "PID.h"
#include <limits>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;

	i_error = 0;
	last_cte = numeric_limits<double>::infinity();
}

void PID::UpdateError(double cte) {
	if (last_cte == numeric_limits<double>::infinity()){
		last_cte = cte;
	}
	p_error = Kp * cte;
	i_error += Ki * cte;
	d_error = Kd * (cte-last_cte);
	last_cte = cte;
}

double PID::TotalError() {
	return p_error + i_error + d_error;
}

