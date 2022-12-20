#include "skill_goto.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>


float euclidean_distance (const position &a, const position &b)
{
    return std::sqrt(std::pow(a.x-b.x, 2)+std::pow(a.y-b.y, 2) + std::pow(a.z-b.z, 2));
}

float euclidean_distance (const position &a)
{
    //std::cout << "Deviation.z: " << a.z << std::endl;
    return std::sqrt(std::pow(a.x, 2)+std::pow(a.y, 2) + std::pow(a.z, 2));
}

// between (-PI, PI)
float normalize_angle(float ang)
{
    //float result = ang;

    if (ang > M_PI)
    {
        while(ang > M_PI)
            ang -= M_PI * 2;
    }
    else if (ang < -M_PI)
    {
        while(ang < -M_PI)
            ang += M_PI * 2;
    }

    return ang;
}


float normalize_angle_diff_yaw(const orientation & target, const orientation & source)
{
    return normalize_angle(target.yaw - source.yaw);
}



Skill_GoToPose::Skill_GoToPose()
{
    action_vw  = new Act_SetVW();
    action_vw->start();

    state_machine       = _Stage_Idle;  // unabled
    _time_loops_elapsed = 0;

    _result.isValid     = false;        //result invalid

    //timer_run = new QTimer();
    //QObject::connect(this->timer_run, SIGNAL(timeout()), this, SLOT(slot_run()));
    
    /// Timer for wdog for stage "Move_END"
    timer_move_end = new QTimer();
    timer_move_end->setInterval(240000);
    timer_move_end->stop();
    QObject::connect(this->timer_move_end, SIGNAL(timeout()), this, SLOT(slot_timeout_ME()));
}

void Skill_GoToPose::set_timers()
{
    //Interval move end
    timer_move_end->setInterval(this->_params.Move_End_Time*1000);  
}

Skill_GoToPose::~Skill_GoToPose()
{
    //delete timer_run;
    delete timer_move_end;
    delete action_vw;
}

void Skill_GoToPose::slot_timeout_ME()
{
    if (state_machine == _Stage_Move_End)
    {
        state_machine = _Stage_Rotate_Target;
    }
    return;
}

Sk_Status_t Skill_GoToPose::run_init()
{
    state_machine       = _Stage_Init;     //innitialized
    _time_loops_elapsed = 0;
    return Sk_Status_ConcludedWithSuccess;
}
Sk_Status_t Skill_GoToPose::run_finish()
{
    state_machine = _Stage_Idle;    // unabled
 
    Act_SetVW_Goal_t tmp_target;
    tmp_target.angular              = 0.0;
    tmp_target.linear               = 0.0;

    action_vw->set_target(tmp_target);
    if (action_vw->get_status() == Status_Idle)
    {
        action_vw->start();
    }

    _actions_exec();
    action_vw->stop();

    std::cout << "Final Skill" << std::endl;

    return Sk_Status_ConcludedWithSuccess;
}

Sk_Status_t Skill_GoToPose::run_loop()
{
    _diagnostics.message = "Skill_GoToPose:: Init state_machine";
    _diagnostics.mode = (int) state_machine;

    /// Set target Goal
    Act_SetVW_Goal_t tmp_target;
    tmp_target.angular              = 0.0;
    tmp_target.linear               = 0.0;

    /// Calc deviation
    _deviation                  = _target - _source;
    _deviation._orientation.yaw = normalize_angle(std::atan2(_deviation._position.y, _deviation._position.x) - _source._orientation.yaw);
    
    /// State machine
    switch(state_machine)
    {
        case _Stage_Init:
        {
            _result.isValid = false;

            // setting parameters of ALL actions based on the current configuration of the skill.
            action_vw->set_params(this->_params.params_action_vw);
            //action_xpto.set_params(this->_params.params_action_xpto);
            //others...
            //_actions_exec();

            if(euclidean_distance(_deviation._position) < _params.D_Finish)
            {
                state_machine = _Stage_Move_End;
                timer_move_end->start();
            }
            else
            {
                state_machine = _Stage_Rotate_Target;
            }

            std::cout<< "Next state: "<<state_machine<<std::endl;
            return Sk_Status_Running;
        } break;

        case _Stage_Rotate_Target:
        {
            // define the desired velocity (wanted)
            float angular_vel               = _deviation._orientation.yaw/M_PI*_params.W_MAX;
            

            if ((angular_vel > 0 ) && (angular_vel < _params.W_MIN ) )
            {
                angular_vel =  _params.W_MIN;
            }
    
            if ((angular_vel < 0 ) && (angular_vel > -_params.W_MIN ) )
            {
                angular_vel =  -_params.W_MIN;
            }       
            

            tmp_target.angular              = angular_vel;
            tmp_target.linear               = 0.0;

            action_vw->set_target(tmp_target);
            // Set velocity
            if (action_vw->get_status() == Status_Idle)
            {
                action_vw->start();
            }
            
            if(std::fabs(_deviation._orientation.yaw) < 2*_params.EPS_pose._orientation.yaw)
            {
                state_machine = _Stage_Move_Start;
                _time_loops_elapsed = 0;
                std::cout<< "Next state:"<<state_machine<<std::endl;
            }
            else
                state_machine = _Stage_Rotate_Target;
            
            _actions_exec();
            return Sk_Status_Running;
        } break;

        case _Stage_Move_Start:
        {
            // define the desired velocity (wanted)
            tmp_target.angular              = _deviation._orientation.yaw/M_PI*_params.W_MAX * _params.W_active;
            tmp_target.linear               = 0;  
            action_vw->set_target(tmp_target);
            // Set velocity
            if (action_vw->get_status() == Status_Idle)
            {
                action_vw->start();
            }
            
            _time_loops_elapsed++;  //elapsing time
            if((_time_loops_elapsed > _params.thrs_time_start_move) || (std::fabs(_deviation._orientation.yaw) < _params.EPS_pose._orientation.yaw))
            {
                if(euclidean_distance(_deviation._position) < _params.D_Finish)
                {
                    state_machine = _Stage_Move_End;
                    timer_move_end->start();
                }
                else
                {
                    state_machine = _Stage_Move_Cruse;
                }

                std::cout<< "Next State:"<<state_machine<<std::endl;
            }
            else 
                state_machine = _Stage_Move_Start;
            
            _actions_exec();
            return Sk_Status_Running;
        } break;

        case _Stage_Move_Cruse:
        {
            // define the desired velocity (wanted)
            tmp_target.angular              = 4*_deviation._orientation.yaw/M_PI*_params.W_MAX;
            tmp_target.linear               = _params.V_MAX;  //const velocity
            action_vw->set_target(tmp_target);
            // Set velocity
            if (action_vw->get_status() == Status_Idle)
            {
                action_vw->start();
            }

            if(euclidean_distance(_deviation._position) < _params.D_Finish)
            {
                state_machine = _Stage_Move_End;
                std::cout<< "Next state:"<<state_machine<<std::endl;
                timer_move_end->start();
            }
            else
                state_machine = _Stage_Move_Cruse;
            
            _actions_exec();
            return Sk_Status_Running;
        } break;

        case _Stage_Move_End:
        {
            // define the desired velocity (wanted)
            tmp_target.angular              = _deviation._orientation.yaw/M_PI*_params.W_MIN;
            tmp_target.linear               = _params.V_MIN;  //const velocity

            action_vw->set_target(tmp_target);
            // Set velocity
            if (action_vw->get_status() == Status_Idle)
            {
                action_vw->start();
            }
            
            _actions_exec();

            if(euclidean_distance(_deviation._position) < _params.EPS_pose._position.x)
            {    
                state_machine = _Stage_Idle;
                std::cout<< "Next state:"<<state_machine<<std::endl;
            
                return Sk_Status_ConcludedWithSuccess;
            }
            
            else if((euclidean_distance(_deviation._position) > (_params.D_Finish + 1.5 )))
            {
                state_machine = _Stage_Rotate_Target;
                std::cout<< "Next state:"<<state_machine<<std::endl;
            }
            else
                state_machine = _Stage_Move_End;
            
            return Sk_Status_Running;
        } break;

        case _Stage_Completed:
        {
            tmp_target.angular              = 0.0;
            tmp_target.linear               = 0.0;

            action_vw->set_target(tmp_target);
            if (action_vw->get_status() == Status_Idle)
            {
                action_vw->start();
            }

            _actions_exec();

            state_machine = _Stage_Idle;

            calc_result();

            return Sk_Status_ConcludedWithSuccess;
        } break;
        
        default:
            return Sk_Status_ConcludedWithoutSuccess;
    }
}


void Skill_GoToPose::calc_result()
{
    if(state_machine == _Stage_Completed)
        _result.isValid = true;

    _deviation                  = _target - _source;
    _deviation._orientation.yaw = normalize_angle(_deviation._orientation.yaw);
    _result.pose_achieved       = _source;
    _result.pose_error          = _deviation;
    return;
}

void Skill_GoToPose::_actions_exec()
{
    //Run action
    action_vw->run();
}