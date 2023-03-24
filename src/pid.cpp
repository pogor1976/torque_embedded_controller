#include "../include/pid.hpp"

PIDcontroller::PIDcontroller(double k_p, double k_i, double k_d,
                             double k_f, double k_b, double torque_limit,
                             double t_f, double k_t_f) : filter(k_t_f)
{
    this->k_p = k_p;
    this->k_i = k_i;
    this->k_d = k_d;
    this->k_f = k_f;
    this->k_b = k_b;
    this->torque_limit = torque_limit;
    this->t_f = t_f;
    // filter = std::make_shared<Filter>(k_t_f);

    prev_time = 0.0;
    i_term = 0.0;
    filter_i_term = 0.0;
}

void PIDcontroller::updateControllerState(double target_value,
                                          double signal_value,
                                          double current_time,
                                          bool is_reset)
{
    double delta_time = current_time - prev_time;
    double error = target_value - input_filter_state;
    // If switch on input filter
    // double error = filter.getFilterOutput() - input_filter_state;
    double no_saturation_control_value = k_p * error + i_term + k_f * (k_d * error - filter_i_term);
    controller_value = std::min(torque_limit, std::max(no_saturation_control_value, -torque_limit));

    //Next state calculation
    input_filter_state += delta_time / t_f * (signal_value - input_filter_state);
    // If switch on input filter
    // filter.updateFilterState(target_value, delta_time);
    i_term += (k_i * error + k_b * (controller_value - no_saturation_control_value)) * delta_time;
    filter_i_term += k_f * (k_d * error - filter_i_term) * delta_time;

    if (is_reset)
    {
        i_term = 0.0;
        filter_i_term = 0.0;
        controller_value = 0.0;
        // If switch on input filter
        // filter.resetFilter();
    }
    prev_time = current_time;
}

double PIDcontroller::getControllerOutput()
{
    return controller_value;
}