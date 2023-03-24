//In this class define filter with transform function: W(s) = ks^3 / (s + ks^3)
#pragma once

class Filter
{
public:
    Filter(double kf);
    void updateFilterState(double input_value, double dt);
    double getFilterOutput();
    void resetFilter();
private:
    double x[3];
    double coeff_a0;
    double coeff_a1;
    double coeff_a2;
    double coeff_b0;
    double output_filter_value;
};