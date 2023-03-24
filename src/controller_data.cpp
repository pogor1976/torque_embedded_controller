#include "../include/controller_data.hpp"
#include <cmath>

SteeringAngleTransform::SteeringAngleTransform(std::vector<double> steering_curve_x_in,
            std::vector<double> steering_curve_y_in,
            ChassisInfo chassis_info_in) :  steering_curve_x(steering_curve_x_in),
                                            steering_curve_y(steering_curve_y_in),
                                            chassis_info(chassis_info_in)
{

}

double SteeringAngleTransform::motorAngle2WheelAngle(double steering_motor_angle)
{
    double steering_rack = steering_motor_angle * chassis_info.steering_rack_reduction;
    steering_rack = std::min(std::max(steering_curve_x.front(), steering_rack), steering_curve_x.back());
    auto i = lower_bound(steering_curve_x.begin(), steering_curve_x.end(), steering_rack);
    auto first_index = std::distance(steering_curve_x.begin(), i);
    if (first_index>0) 
    {
        first_index--;
    }
    else
    {   
        first_index = 0;
    }
    auto end_index = first_index + 1;
    if (end_index > steering_curve_x.size())
    {
        return steering_curve_x.at(first_index);
    }
    else
    {
        // Linear interpolation
        return (steering_curve_y.at(first_index) +
               (steering_curve_y.at(end_index) - steering_curve_y.at(first_index)) /
               (steering_curve_x.at(end_index) - steering_curve_x.at(first_index)) *
               (steering_rack - steering_curve_x.at(first_index)));
    }

    return 0.0;
}

void SteeringAngleTransform::motorAngle2WheelAngles(double steering_motor_angle, double wheel_angles[2])
{
    double middle_angle = steering_motor_angle * chassis_info.steering_motor_angle_to_middle_angle_reduction;
    middle_angle = std::min(std::max(middle_angle, -chassis_info.max_middle_angle), chassis_info.max_middle_angle);
    double angle_abs = fabs(middle_angle);
    double tan_angle = tan(angle_abs);
    double alpha1 = atan(chassis_info.wheel_base * tan_angle / 
                         (chassis_info.wheel_base - chassis_info.track / 2 * tan_angle));
    double alpha2 = atan(chassis_info.wheel_base * tan_angle / 
                         (chassis_info.wheel_base + chassis_info.track / 2 * tan_angle));
    if (middle_angle > 0)
    {
        wheel_angles[0] = -alpha2;
        wheel_angles[1] = -alpha1;
    }
    else
    {
        wheel_angles[0] = alpha1;
        wheel_angles[1] = alpha2;
    }
}
