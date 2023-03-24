#include <iostream>
#include "../include/controller_data.hpp"
#include "../include/pid.hpp"
#include "../include/sweeper_chassis_controller.hpp"
#include <cmath>

int main()
{
    //Steering transform data
    std::vector<double> vec_x = {-0.025, -0.02236842, -0.01973684, -0.01710526, -0.01447368, -0.01184211,
                                 -0.00921053, -0.00657895, -0.00394737, -0.00131579,  0.00131579,  0.00394737,
                                 0.00657895,  0.00921053,  0.01184211,  0.01447368,  0.01710526,  0.01973684,
                                 0.02236842,  0.025};
    std::vector<double> vec_y = {-27.06661054, -24.25486289, -21.4496171,  -18.64435009, -15.8328788,
                                 -13.00923999, -10.16758536,  -7.30208602,  -4.40684142,  -1.47578848,
                                 1.4973936,    4.51938725,   7.59735461,  10.73907656,  13.95312166,
                                 17.24905714,  20.63771977,  24.13157237,  27.74518551,  31.49590645};
    ChassisInfo chassis_info = {0.025, //drive_motor_reduction
                                0.00024867959858108647, //steering_rack_reduction
                                0.165, //wheel_radius
                                24.5 * M_PI / 180.0, //max_middle_angle
                                0.65, //wheel_base
                                0.54, //track
                                1.0 / 40 / 4.9655 //steering_motor_angle_to_middle_angle_reduction
                                };
    SteeringAngleTransform st_angle_tr(vec_x, vec_y, chassis_info);

    //Motor controller data
    PIDcontroller left_motor_controller(120.0, //k_p
                                        240.0, //k_i
                                        0.0, //k_d
                                        0.0, //k_f
                                        10.0, //k_b
                                        300.0, //torque_limit
                                        0.05, //t_f
                                        2.5 //k_t_f
                                        );
    PIDcontroller right_motor_controller(120.0, //k_p
                                        240.0, //k_i
                                        0.0, //k_d
                                        0.0, //k_f
                                        10.0, //k_b
                                        300.0, //torque_limit
                                        0.05, //t_f
                                        2.5 //k_t_f
                                        );
    // SweeperChassisController sweeper_chassis_cotroller(left_motor_controller, right_motor_controller, st_angle_tr);

    std::shared_ptr<SweeperChassisController> sweeper_chassis_cotroller;
    sweeper_chassis_cotroller = std::shared_ptr<SweeperChassisController>(new SweeperChassisController(left_motor_controller, right_motor_controller, st_angle_tr));
    double model_time = 0.0;
    double dt = 0.01;
    for(int i=0; i<100; i++)
    {
        //Input variables
        double left_front_wheel_target_motor_speed = 400.0;
        double right_front_wheel_target_motor_speed = 400.0;
        double steering_motor_target_angle = 100.0;
        double left_front_wheel_current_motor_speed = 0.0;
        double right_front_wheel_current_motor_speed = 0.0;

        //Update controller state
        sweeper_chassis_cotroller->updateControllerState(model_time,
                                                         left_front_wheel_target_motor_speed,
                                                         left_front_wheel_current_motor_speed,
                                                         right_front_wheel_target_motor_speed,
                                                         right_front_wheel_current_motor_speed,
                                                         steering_motor_target_angle);

        //Get wheel states.
        std::vector<WheelState>* wheels = sweeper_chassis_cotroller->getWheelStates();
        double left_steering_angle = wheels->at(2).get_target_steering_angle();
        double right_steering_angle = wheels->at(3).get_target_steering_angle();
        double left_target_torque = wheels->at(0).get_target_torque();
        double right_target_torque = wheels->at(1).get_target_torque();

        //Do something with them
        std::cout << "steering left wheel angle: " << (left_steering_angle * 180.0 / M_PI) << ", deg" << std::endl;
        std::cout << "steering right wheel angle: " << (right_steering_angle * 180.0 / M_PI) << ", deg" << std::endl; 
        std::cout << "t: " << model_time << ", left_target_torque: " << left_target_torque << std::endl;
        std::cout << "t: " << model_time << ", right_target_torque: " << right_target_torque << std::endl;

        //Change model time
        model_time = model_time + dt;
    }

    return 0;
}
