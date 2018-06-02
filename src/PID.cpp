#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::init(const double Kp, const double Ap, const double Ki, const double Ai, const double Kd, const double Ad, double const Wi) {
    err_p_ = 0;
    err_i_ = 0;
    err_d_ = 0;
    err_total_ = 0;

    Kp_ = Kp;
    Ki_ = Ki;
    Kd_ = Kd;

    // Modify the weights linearly based on a second variable.
    // If set to 0, this will work like a conventional PID controller.
    Ap_ = Ap;
    Ai_ = Ai;
    Ad_ = Ad;

    // Make the integral part of the process variable vanish overtime:
    Wi_ = Wi;

    pv_prev_ = 0;
    pv_int_ = 0;
}

double PID::update(const double pv, const double alpha, const double min, const double max) {

    // Calculate the change in CTE and the accumulated CTE (weighted):
    const double pv_diff = pv - pv_prev_;

    // Update the previous CTE to calculate the diff in the next timestep:
    pv_prev_ = pv;

    pv_int_ = pv + Wi_ * pv_int_;

    // Calculate the contribution of each component to the total erro:
    err_p_ = (-Kp_ - Ap_ * alpha) * pv;
    err_i_ = (-Ki_ - Ai_ * alpha) * pv_int_;
    err_d_ = (-Kd_ - Ad_ * alpha) * pv_diff;

    // Calculate the total error:
    err_total_ = err_p_ + err_i_ + err_d_;

    // Return min <= err_total_ <= max:
    // Note that the original err_total_ value can be accessed.

    if (err_total_ < min) {
        return min;
    }

    if (err_total_ > max) {
        return max;
    }

    return err_total_;
}

double PID::getTotalError() {
    return err_total_;
}

