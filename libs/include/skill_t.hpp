#ifndef SKILL_T_HPP
#define SKILL_T_HPP

#include <iostream>


//! Sk_Goal_t template for tasks:
/*!
 * Value (the type for a numeric variable)
 * Data (the type of a composed variable, eg. a structure)
 *
 *
 *  EXAMPLES:
 *
 *  // Numeric values
    Sk_Goal_t<int, float> velocity;
    velocity.value = 1;
    std::cout<<" Value:"<<velocity.value<<std::endl;


    // With Composed data
    struct structure_val
    {
        float x;
        float y;
        float z;
    };
    Sk_Goal_t<int, structure_val> velocity_point;
    velocity_point.data.x = 1.0;
    velocity_point.data.y = 2.1;
    velocity_point.data.z = 3.2;
 *
 *
 */
template<typename Value, typename Data> struct Sk_Goal_t
{
    Value value = 0;
    Data data;
};


//! Sk_Params_t template for providing parameters to the task.
/*!
 * Data (can be a composed variable, eg. a structure)
 *
 */
template<typename Data> struct Sk_Params_t
{
    Data data;
};



//! Sk_Result_t template for reporting the result of the task after conclusion.
/*!
 * Data (can be a composed variable, eg. a structure)
 *
 */
template<typename Data> struct Sk_Result_t
{
    Data data;
};



//! Diagnostic_t template for reporting the "fault".
/*!
 * Data (can be a composed variable, eg. a structure)
 *
 */
template<typename Data> struct Sk_Diagnostics_t
{
    Data data;
};




//! Modes for the "health"/status of the task for monitoring
enum Sk_Status_t
{
    Sk_Status_Idle = 0,
    Sk_Status_ReadyToStart = 1,
    Sk_Status_Running = 2,
    Sk_Status_Paused = 3,
    Sk_Status_ConcludedWithSuccess = 4,
    Sk_Status_ConcludedWithoutSuccess = 5,
    Sk_Status_Stopped = 6,
    Sk_Status_Aborting = 7,
    Sk_Status_Error = 8
};


//! Template class for "task"
/*!
* typedef Sk_Goal_t<int,int> GOAL;
* ...
* Skill_t<GOAL, GOAL, GOAL, GOAL>  task;
* task.print();
*
*/

template <class Sk_Params_t, class Sk_Goal_t, class Sk_Diagnostics_t, class Sk_Result_t>
class Skill_t
{

protected:
    Sk_Goal_t _target;                     //!< Desired target for completion.
    Sk_Goal_t _source;                     //!< Current state of the goal.
    Sk_Goal_t _deviation;                  //!< Deviation between source to target (if needed).

    Sk_Params_t _params;                   //!< Parameters required by the task.
    Sk_Status_t _status;                   //!< Current status of the task.
    Sk_Diagnostics_t _diagnostics;         //!< Diagnostic feedback: self monitoring mechanism.

    Sk_Result_t _result;                   //!< Result of the processing.


    ///!< Routines that need to be overwritted.
    virtual Sk_Status_t run_init() = 0;        //!< Runs only once (at start). Returns _status.
    /* returns: Status_Running or Status_ConcludedWithSuccess or Status_ConcludedWithoutSuccess */

    virtual Sk_Status_t run_loop() = 0;        //!< Runs continuously. Returns _status.
    /* returns: Status_Running or Status_ConcludedWithSuccess or Status_ConcludedWithoutSuccess */

    virtual Sk_Status_t run_finish() = 0;      //!< Runs only once (at finish). Returns _status.
    /* returns: Status_Running or Status_ConcludedWithSuccess or Status_ConcludedWithoutSuccess */

    virtual void calc_result() = 0;        //!< translates and saves the result of the "run_init"+"run_loop"+"run_finish".
    /* saves: the output to be provided to skill in variable '_result' */

    //TO BE IMPLEMENTED
    virtual Sk_Diagnostics_t  run_fault_monitoring(); //!< Runs the diagnostic module.
    /* returns: Diagnostic module for the skill */

public:
    Skill_t();

    Sk_Status_t run();                             //!< Non-blocking! member that should be executed by the Skill for running this task.
    Sk_Status_t stop();                            //!< member that should be executed by the Skill for stopping this task.
    Sk_Status_t start();

    void set_source(const Sk_Goal_t &src);         //!< member that should be executed by the Skill for setting the source for the Goal.
    void set_target(const Sk_Goal_t &src);         //!< member that should be executed by the Skill for setting the target for the Goal.
    void set_params(const Sk_Params_t & src );     //!< member that should be executed by the Skill for setting the parameters that will be used in "run_xpto".
    Sk_Params_t get_params();                      //!< member that should be executed by the Skill for getting the parameters.

    Sk_Status_t get_status();                      //!< member that should be executed by the Skill for getting the status of task.
    Sk_Diagnostics_t get_diagnostics();            //!< member that should be executed by the Skill for getting the diagnostics that will be updated in "run_xpto".
    Sk_Result_t get_result();                      //!< member that should be executed by the Skill for getting the result that is obtained by the ''calc_result''

    void print_debug();
};






/// Constructor
template<class Sk_Params_t, class Sk_Goal_t, class Sk_Diagnostics_t, class Sk_Result_t>
Skill_t<Sk_Params_t, Sk_Goal_t, Sk_Diagnostics_t, Sk_Result_t>::Skill_t()
{
    _status = Sk_Status_Idle;
    return;
}


/// Mechanism for running the task.
template<class Sk_Params_t, class Sk_Goal_t, class Sk_Diagnostics_t, class Sk_Result_t>
Sk_Status_t Skill_t<Sk_Params_t, Sk_Goal_t, Sk_Diagnostics_t, Sk_Result_t>::run()
{
    Sk_Status_t temp;

    switch(_status)
    {

    // run init
    case Sk_Status_ReadyToStart:

        temp = run_init();
        _deviation = _target - _source;

        if ( temp == Sk_Status_Running )
            return Sk_Status_Running;
        if ( temp == Sk_Status_ConcludedWithoutSuccess )
        {
            _status = Sk_Status_Stopped;
            return Sk_Status_ConcludedWithoutSuccess;
        }
        if ( temp == Sk_Status_ConcludedWithSuccess )
        {
            //Status_ConcludedWithSuccess --> it can proceed
            _status = Sk_Status_Running;
            return Sk_Status_Running;
        }

        break;

        // run loop
    case Sk_Status_Running:

        temp = run_loop();
        _deviation = _target - _source;

        if ( temp == Sk_Status_ConcludedWithSuccess )
        {
            //Status_ConcludedWithSuccess --> it can proceed.
            _status = Sk_Status_ConcludedWithSuccess;
            return Sk_Status_Running;
        }

        if ( temp == Sk_Status_Running )
            return Sk_Status_Running;
        
        if ( temp == Sk_Status_ConcludedWithoutSuccess )
        {
            _status = Sk_Status_Stopped;
            return Sk_Status_ConcludedWithoutSuccess;
        }
        
        break;

        // run finish
    case Sk_Status_ConcludedWithSuccess:
        temp = run_finish();
        _deviation = _target - _source;

        if ( temp == Sk_Status_Running )
        {
            return Sk_Status_Running;
        }
        if ( temp == Sk_Status_ConcludedWithoutSuccess )
        {
            _status = Sk_Status_Stopped;
            return Sk_Status_ConcludedWithoutSuccess;
        }
        if ( temp == Sk_Status_ConcludedWithSuccess )
        {
            _status = Sk_Status_Idle;
            return Sk_Status_ConcludedWithSuccess;
        }
        break;

    default:
        return _status;
    }

}

template<class Sk_Params_t, class Sk_Goal_t, class Sk_Diagnostics_t, class Sk_Result_t>
Sk_Status_t Skill_t<Sk_Params_t, Sk_Goal_t, Sk_Diagnostics_t, Sk_Result_t>::stop()
{
    _status = Sk_Status_Stopped;
    return _status;
}

template<class Sk_Params_t, class Sk_Goal_t, class Sk_Diagnostics_t, class Sk_Result_t>
Sk_Status_t Skill_t<Sk_Params_t, Sk_Goal_t, Sk_Diagnostics_t, Sk_Result_t>::start()
{
    _status = Sk_Status_ReadyToStart;
    return _status;
}


template<class Sk_Params_t, class Sk_Goal_t, class Sk_Diagnostics_t, class Sk_Result_t>
void Skill_t<Sk_Params_t, Sk_Goal_t, Sk_Diagnostics_t, Sk_Result_t>::set_source(const Sk_Goal_t &src)
{
    _source = src;
    return;

}

template<class Sk_Params_t, class Sk_Goal_t, class Sk_Diagnostics_t, class Sk_Result_t>
void Skill_t<Sk_Params_t, Sk_Goal_t, Sk_Diagnostics_t, Sk_Result_t>::set_target(const Sk_Goal_t &src)
{
    _target = src;
    return;
}

template<class Sk_Params_t, class Sk_Goal_t, class Sk_Diagnostics_t, class Sk_Result_t>
void Skill_t<Sk_Params_t, Sk_Goal_t, Sk_Diagnostics_t, Sk_Result_t>::set_params(const Sk_Params_t &src)
{
    _params = src;
    return;
}


template<class Sk_Params_t, class Sk_Goal_t, class Sk_Diagnostics_t, class Sk_Result_t>
Sk_Params_t Skill_t<Sk_Params_t, Sk_Goal_t, Sk_Diagnostics_t, Sk_Result_t>::get_params()
{
    return _params;
}





template<class Sk_Params_t, class Sk_Goal_t, class Sk_Diagnostics_t, class Sk_Result_t>
Sk_Status_t Skill_t<Sk_Params_t, Sk_Goal_t, Sk_Diagnostics_t, Sk_Result_t>::get_status()
{
    return _status;
}

template<class Sk_Params_t, class Sk_Goal_t, class Sk_Diagnostics_t, class Sk_Result_t>
Sk_Diagnostics_t Skill_t<Sk_Params_t, Sk_Goal_t, Sk_Diagnostics_t, Sk_Result_t>::get_diagnostics()
{
    return _diagnostics;
}

template<class Sk_Params_t, class Sk_Goal_t, class Sk_Diagnostics_t, class Sk_Result_t>
Sk_Result_t Skill_t<Sk_Params_t, Sk_Goal_t, Sk_Diagnostics_t, Sk_Result_t>::get_result()
{
    return _result;
}

template<class Sk_Params_t, class Sk_Goal_t, class Sk_Diagnostics_t, class Sk_Result_t>
void Skill_t<Sk_Params_t, Sk_Goal_t, Sk_Diagnostics_t, Sk_Result_t>::print_debug()
{
    std::cout<<" Task_ print()"<<std::endl;
    return;
}


/* ************************************************************************** */
/* ************************************************************************** */
/*    Preparing the routines for the task                                   */
/* ************************************************************************** */
/* ************************************************************************** */
/// To be Overridden
/*
template<class Sk_Params_t, class Sk_Goal_t, class Sk_Diagnostics_t, class Sk_Result_t>
Sk_Status_t Skill_t<Sk_Params_t, Sk_Goal_t, Sk_Diagnostics_t, Sk_Result_t>::run_init(){}

/// To be Overridden
template<class Sk_Params_t, class Sk_Goal_t, class Sk_Diagnostics_t, class Sk_Result_t>
Sk_Status_t Skill_t<Sk_Params_t, Sk_Goal_t, Sk_Diagnostics_t, Sk_Result_t>::run_loop(){}

/// To be Overridden
template<class Sk_Params_t, class Sk_Goal_t, class Sk_Diagnostics_t, class Sk_Result_t>
Sk_Status_t Skill_t<Sk_Params_t, Sk_Goal_t, Sk_Diagnostics_t, Sk_Result_t>::run_finish(){}


/// To be Overridden
template<class Sk_Params_t, class Sk_Goal_t, class Sk_Diagnostics_t, class Sk_Result_t>
void Skill_t<Sk_Params_t, Sk_Goal_t, Sk_Diagnostics_t, Sk_Result_t>::calc_result()
{
    //_result = .... ;
    return;
}
*/

/// To be Overridden AND IMPLEMENTED
template<class Sk_Params_t, class Sk_Goal_t, class Sk_Diagnostics_t, class Sk_Result_t>
Sk_Diagnostics_t Skill_t<Sk_Params_t, Sk_Goal_t, Sk_Diagnostics_t, Sk_Result_t>::run_fault_monitoring(){}

#endif


