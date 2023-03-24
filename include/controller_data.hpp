#pragma once
#include <vector>

enum MotorCommandType
{
    RPM_CMD = 1,
    POSE_CMD = 2,
    ORQUE_CMD = 3
};

enum WheelControlType
{
    TORQUE_CONTROL,
    ANGLE_CONTROL
};

struct MotorCommand
{
    MotorCommandType command_type;
    double torque;
    double speed;
    double angle;
};

struct ChassisInfo
{
    double drive_motor_reduction;
    double steering_rack_reduction;
    double wheel_radius;
    double max_middle_angle;
    double wheel_base;
    double track;
    double steering_motor_angle_to_middle_angle_reduction;
};

class SteeringAngleTransform
{
public:
    SteeringAngleTransform(std::vector<double> steering_curve_x_in,
                           std::vector<double> steering_curve_y_in,
                           ChassisInfo chassis_info_in);
    double motorAngle2WheelAngle(double steering_motor_angle);
    void motorAngle2WheelAngles(double steering_motor_angle, double wheel_angles[]);

    // Access functions
    ChassisInfo* getChassisInfo(){ return (&chassis_info); }
private:
    std::vector<double> steering_curve_x;
    std::vector<double> steering_curve_y;
    ChassisInfo chassis_info;
};

class WheelState
{
public:
    WheelState(int wheel_index, WheelControlType control_type) : 
        index(wheel_index), type(control_type)
    {

    }
public:
    // Access functions
    double get_target_torque() { return target_torque; };
    void set_target_torque(double value) { target_torque = value; }
    double get_target_angular_speed() { return target_angular_speed; }
    void set_target_angular_speed(double value) { target_angular_speed = value; }
    double get_target_steering_angle() { return target_steering_angle; }
    void set_target_steering_angle(double value) {target_steering_angle = value; }

    double get_current_torque() { return current_torque; };
    void set_current_torque(double value) { current_torque = value; }
    double get_current_angular_speed() { return current_angular_speed; }
    void set_current_angular_speed(double value) { current_angular_speed = value; }
    double get_current_steering_angle() { return current_steering_angle; }
    void set_current_steering_angle(double value) {current_steering_angle = value; }

private:
    int index;
    WheelControlType type;
    double target_torque;
    double target_angular_speed;
    double target_steering_angle;
    double current_torque;
    double current_angular_speed;
    double current_steering_angle;
};
