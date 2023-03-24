#include "../include/filter.hpp"
#include <cmath>

Filter::Filter(double kf)
{
    coeff_a0 = pow(kf, 3.0);
    coeff_a1 = 3 * pow(kf, 2.0);
    coeff_a2 = 3 * kf;
    coeff_b0 = coeff_a0; 
    resetFilter();
}

void Filter::updateFilterState(double input_value, double dt)
{
    double x0 = x[0] + dt * x[1];
    double x1 = x[1] + dt * x[2];
    double x2 = x[2] + dt * (coeff_a0 * x[0] + coeff_a1 * x[1] + coeff_a2 * x[2] + input_value);
    output_filter_value = coeff_b0 * x[0];

    x[0] = x0;
    x[1] = x1;
    x[2] = x2;
}

double Filter::getFilterOutput()
{
    return output_filter_value;
}

void Filter::resetFilter()
{
    x[0] = 0.0;
    x[1] = 0.0;
    x[2] = 0.0;
}
