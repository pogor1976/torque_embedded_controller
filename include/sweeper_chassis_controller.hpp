#pragma once

#include "../include/controller_data.hpp"
#include "../include/pid.hpp"

class SweeperChassisController
{
public:
    SweeperChassisController(PIDcontroller left_motor_controller_in,
                             PIDcontroller right_motor_controller_in,
                             SteeringAngleTransform steering_angle_transform_in);
    void updateControllerState(double model_time,
                               double left_motor_target_angular_speed,
                               double left_wheel_current_angular_speed,
                               double right_motor_target_angular_speed,
                               double right_wheel_current_angular_speed,
                               double steering_motor_angle);
    // Access function;
    std::vector<WheelState>* getWheelStates(){ return (&wheels); }
private:
    PIDcontroller left_motor_controller;
    PIDcontroller right_motor_controller;
    SteeringAngleTransform steering_angle_transform;

    std::vector<WheelState> wheels;
};
