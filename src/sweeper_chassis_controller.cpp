#include "../include/sweeper_chassis_controller.hpp"
#include <cmath>

SweeperChassisController::SweeperChassisController(
                    PIDcontroller left_motor_controller_in,
                    PIDcontroller right_motor_controller_in,
                    SteeringAngleTransform steering_angle_transform_in):
                    left_motor_controller(left_motor_controller_in),
                    right_motor_controller(right_motor_controller_in),
                    steering_angle_transform(steering_angle_transform_in)
{
    wheels.push_back(WheelState(0, WheelControlType::TORQUE_CONTROL));
    wheels.push_back(WheelState(1, WheelControlType::TORQUE_CONTROL));
    wheels.push_back(WheelState(2, WheelControlType::ANGLE_CONTROL));
    wheels.push_back(WheelState(3, WheelControlType::ANGLE_CONTROL));
}

void SweeperChassisController::updateControllerState(double model_time,
                                                     double left_motor_target_angular_speed,
                                                     double left_wheel_current_angular_speed,
                                                     double right_motor_target_angular_speed,
                                                     double right_wheel_current_angular_speed,
                                                     double steering_motor_angle)
{
    // Change steering angle values
    double wheel_angles[2];
    steering_angle_transform.motorAngle2WheelAngles(steering_motor_angle, wheel_angles);
    wheels[2].set_target_steering_angle(wheel_angles[0]);
    wheels[3].set_target_steering_angle(wheel_angles[1]);
    
    ChassisInfo* ch_inf = steering_angle_transform.getChassisInfo();
    wheels[0].set_target_angular_speed(left_motor_target_angular_speed * ch_inf->drive_motor_reduction);
    wheels[1].set_target_angular_speed(right_motor_target_angular_speed * ch_inf->drive_motor_reduction);
    wheels[0].set_current_angular_speed(left_wheel_current_angular_speed);
    wheels[1].set_current_angular_speed(right_wheel_current_angular_speed);

    double left_drive_wheel_estimation = wheels[0].get_current_angular_speed();
    double right_drive_wheel_estimation = wheels[1].get_current_angular_speed();

    bool is_reset = false;
    // Stop condition   
    // if (fabs(wheels[0].get_current_angular_speed()) < 0.5 and
    //     fabs(wheels[1].get_current_angular_speed()) < 0.5 and
    //     fabs(wheels[0].get_target_angular_speed()) < 0.5 and
    //     fabs(wheels[1].get_current_angular_speed()) < 0.5 and
    //     fabs(left_motor_controller.getControllerOutput()) < 5 and
    //     fabs(right_motor_controller.getControllerOutput()) < 5)
    // {
    //     is_reset = true;
    // }
    
    left_motor_controller.updateControllerState(wheels[0].get_target_angular_speed(),
                                                left_drive_wheel_estimation,
                                                model_time,
                                                is_reset);
    right_motor_controller.updateControllerState(wheels[1].get_target_angular_speed(),
                                                 right_drive_wheel_estimation,
                                                 model_time,
                                                 is_reset);
    wheels[0].set_target_torque(left_motor_controller.getControllerOutput());
    wheels[1].set_target_torque(right_motor_controller.getControllerOutput());

    
}