#include "action_setvw.hpp"
#include <cmath>

#define EPS_angular 0.01
#define EPS_linear 0.005

Status_t Act_SetVW::run_init()
{
    //Velocity limit
    this->desired_velocity.angular = _target.angular;
    this->desired_velocity.linear = _target.linear;

    return Status_ConcludedWithSuccess;
}

Status_t Act_SetVW::run_loop()
{
    // User-defined functions/procedures
    //limit_velocity();                                       //physical limitation of the velocity!

    this->desired_velocity.angular = _target.angular;
    this->desired_velocity.linear = _target.linear;

    if(_target.linear == 0 && _target.angular == 0)
    {
        this->V_left = 0;
        this->V_right = 0;
    }
    else 
    {
        //limitVel(_target.linear, _params.max_linear_speed);
        //limitVel(_target.angular, _params.max_angular_speed);

        ASV_kinematics(this->desired_velocity, this->V_left, this->V_right);
    }

    calc_result();
    
    if ((std::fabs(this->desired_velocity.angular - _target.angular)<EPS_angular) && (std::fabs(this->desired_velocity.linear - _target.linear)<EPS_linear))
        return Status_ConcludedWithSuccess;
    //to be rectified
    else
        return Status_ConcludedWithSuccess;
}

Status_t Act_SetVW::run_finish()
{
    // do  nothing
    this->desired_velocity.angular = _target.angular;
    this->desired_velocity.linear = _target.linear;

    return Status_ConcludedWithSuccess;
}

void Act_SetVW::calc_result()
{
    _result.V_left = V_left;
    _result.V_right = V_right;
    return;
}


void Act_SetVW::ASV_kinematics(const Act_SetVW_Goal_t & desired_velocity, float &V_left, float &V_right)
{
    //Conversion to m/s for each motor
    V_right = _params.linear_scaling*desired_velocity.linear + _params.angular_scaling*desired_velocity.angular;
    V_left = _params.linear_scaling*desired_velocity.linear - _params.angular_scaling*desired_velocity.angular;

    //Get linear and angular velocity ratio
    double alpha_ = 0.0;
    if(desired_velocity.angular == 0.0)
        alpha_ = 1.0;
    else
        alpha_ = std::fabs(desired_velocity.linear/desired_velocity.angular);

    //Get maximum reachable per motor velocity
    double max_motor_vel_ = alpha_*_params.linear_scaling*_params.max_linear_speed + (1.0-alpha_)*_params.angular_scaling*_params.max_linear_speed;

    //Normalize for RPM
    V_left *= (_params.rpm/max_motor_vel_);
    V_right *= (_params.rpm/max_motor_vel_);

    //limit
    V_left = limitVel(V_left, _params.rpm);
    V_right = limitVel(V_right, _params.rpm);

    //Conversion from RPM to rad/s
    V_right *= (2.0*M_PI/60);
    V_left *= (2.0*M_PI/60);
}

double Act_SetVW::limitVel(const double &vel_, const double &max_vel_)
{
    double vel_min_ = std::min(std::fabs(vel_), max_vel_);
    return sgn(vel_) * vel_min_;
}

Act_SetVW::Act_SetVW()
{
    _source.angular = 0;
    _source.linear = 0;
    _target.angular = 0;
    _target.linear = 0;

    desired_velocity.angular = 0;
    desired_velocity.linear  = 0;
    V_right = 0;
    V_left = 0;

    return;
}
