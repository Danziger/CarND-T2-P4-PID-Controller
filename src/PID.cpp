#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::init(const double kp, const double ki, const double kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

double PID::update(const double cte, const double min, const double max) {
    err_total_ += cte * cte;

    const double cte_diff = cte - cte_prev_;

    cte_prev_ = cte;
    cte_int_ += cte;

    const double value = -kp_ * cte - ki_ * cte_int_ - kd_ * cte_diff;

    if (value < min) {
        return min;
    }

    if (value > max) {
        return max;
    }

    return value;
}

double PID::getTotalError() {
    return err_total_;
}

