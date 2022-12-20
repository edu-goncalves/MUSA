#include "task_goto_seq.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>


Task_GoToSequence::Task_GoToSequence()
{
    skill_goto = new Skill_GoToPose();

    _Stages_GoToSequence = -1; //inactive
    //_time_loops_elapsed = 0;

    _result.isValid     = false;        //result invalid

    timer_run = new QTimer();

    QObject::connect(this->timer_run, SIGNAL(timeout()), this, SLOT(slot_run()));
}

Task_GoToSequence::~Task_GoToSequence()
{
    delete timer_run;
    delete skill_goto;
}

void Task_GoToSequence::set_timers()
{
    timer_run->setInterval(_params.params_skill_goto.params_action_vw.delta_time*100);    
}

void Task_GoToSequence::slot_run()
{
    T_Status_t ___status = this->run();

    emit sig_result_vw(skill_goto->action_vw->desired_velocity.linear, skill_goto->action_vw->desired_velocity.angular);
    return;
}


T_Status_t Task_GoToSequence::run_init()
{
    _Stages_GoToSequence = 0;
    _diagnostics.message = "Task Started";
    _diagnostics.mode = _Stages_GoToSequence;
    
    return T_Status_ConcludedWithSuccess;
}

T_Status_t Task_GoToSequence::run_finish()
{
    _Stages_GoToSequence = -1;
    _diagnostics.message = "Task Completed";
    _diagnostics.mode = _Stages_GoToSequence;

    std::cout << _diagnostics.message << std::endl;

    _result.isValid = true;

    return T_Status_ConcludedWithSuccess;
}

T_Status_t Task_GoToSequence::run_loop()
{
    Sk_Status_t sk_status = skill_goto->get_status();
    
    if (_Stages_GoToSequence >= _target._waypoints.size())
    {
        if (sk_status == Sk_Status_ConcludedWithSuccess || sk_status == Sk_Status_Idle)
            return T_Status_ConcludedWithSuccess;
    }

    //if skill is not active send next goal
    else if((sk_status == Sk_Status_Idle) || (sk_status == Sk_Status_Stopped)) 
    { 
        skill_goto->start();
        Skill_GoToPose_Goal_t current_goal = _target._waypoints.at(_Stages_GoToSequence);
        skill_goto->set_target(current_goal);
    }

    _diagnostics.message = "Task Running";
    _diagnostics.mode = _Stages_GoToSequence;

    //execute skill
    if(_skill_exec() == Sk_Status_ConcludedWithSuccess)
    {
        _Stages_GoToSequence++;
        _result.pose_achieved._waypoints.push_back(skill_goto->get_result().pose_achieved);
    }

    return T_Status_Running;    
}

Sk_Status_t Task_GoToSequence::_skill_exec()
{
    //Run skill
    //std::cout << "skill exec" << std::endl;
    return skill_goto->run();
}


void Task_GoToSequence::slot_start_exec(int interval_ms = 1000)
{
    timer_run->setInterval(interval_ms);
    timer_run->start();

    this->start();

    return;
}


void Task_GoToSequence::slot_stop_exec()
{
    this->stop();
    timer_run->stop();
    _Stages_GoToSequence = -1; //inactivate task
    
    Task_GoToSequence_Targets_t aux;
    this->set_target(aux);

    return;
}








