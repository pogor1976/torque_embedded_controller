#pragma once

#include <vector>
#include <memory>
#include "filter.hpp"

class PIDcontroller
{
    // The PID controller with anti-windup mechanism is described in Simulink
    // documantation.
    // Base code from https://github.com/ivmech/ivPID/blob/master/PID.py was adapted.

public:
    // Constructor. 
    // Arguments:
    // k_p - the coefficient of proportial part;
    // k_i - the coefficient of integral part;
    // k_d - the coefficient of differential part;
    // k_f - filter coefficient;
    // k_b - back-calculation coefficient;
    // t_f - parameter of prev filter of feedback signal;
    // k_t_f - parameter of the block limiting the rise of the input signal;
    // torque_limit -  limitation of control torque.

    PIDcontroller(double k_p, double k_i, double k_d,
                  double k_f, double k_b, double torque_limit,
                  double t_f, double k_t_f);
    void updateControllerState(double target_value,
                               double signal_value,
                               double current_time,
                               bool is_reset);
    double getControllerOutput();
private:
    //Parameters
    double k_p;
    double k_i;
    double k_d;
    double k_f;
    double k_b;
    double torque_limit;
    double t_f;
    // std::shared_ptr<Filter> filter;
    Filter filter;
    //Fix time
    double prev_time;
    //Inner states
    double i_term;
    double filter_i_term;
    double input_filter_state;
    //Output controller value
    double controller_value;
};