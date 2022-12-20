#ifndef ACTION_T_HPP
#define ACTION_T_HPP

#include <iostream>


//! Goal_t template for actions:
/*!
 * Value (the type for a numeric variable)
 * Data (the type of a composed variable, eg. a structure)
 *
 *
 *  EXAMPLES:
 *
 *  // Numeric values
    Goal_t<int, float> velocity;
    velocity.value = 1;
    std::cout<<" Value:"<<velocity.value<<std::endl;


    // With Composed data
    struct structure_val
    {
        float x;
        float y;
        float z;
    };
    Goal_t<int, structure_val> velocity_point;
    velocity_point.data.x = 1.0;
    velocity_point.data.y = 2.1;
    velocity_point.data.z = 3.2;
 *
 *
 */
template<typename Value, typename Data> struct Goal_t
{
    Value value = 0;
    Data data;
};


//! Params_t template for providing parameters to the Action.
/*!
 * Data (can be a composed variable, eg. a structure)
 *
*/
template<typename Data> struct Params_t
{
    Data data;
};


//! Result_t template for reporting the result of the action after conclusion.
/*!
 * Data (can be a composed variable, eg. a structure)
 *
 */
template<typename Data> struct Result_t
{
    Data data;
};



//! Diagnostic_t template for reporting the "fault".
/*!
 * Data (can be a composed variable, eg. a structure)
 *
 */
template<typename Data> struct Diagnostics_t
{
    Data data;
};




//! Modes for the "health"/status of the action for monitoring
enum Status_t
{
    Status_Idle = 0,
    Status_ReadyToStart = 1,
    Status_Running = 2,
    Status_Paused = 3,
    Status_ConcludedWithSuccess = 4,
    Status_ConcludedWithoutSuccess = 5,
    Status_Stopped = 6,
    Status_Aborting = 7,
    Status_Error = 8
};


//! Template class for "Action"
/*!
* typedef Goal_t<int,int> GOAL;
* ...
* Action_t<GOAL, GOAL, GOAL, GOAL>  action;
* action.print();
*
*/
template <class Params_t, class Goal_t, class Diagnostics_t, class Result_t>
class Action_t
{

private:
    Status_t _status;                   //!< Current status of the action.
protected:
    Goal_t _target;                     //!< Desired target for completion.
    Goal_t _source;                     //!< Current state of the goal.
    Goal_t _deviation;                  //!< Deviation between source to target (if needed).

    Params_t _params;                   //!< Parameters required by the action.

    Diagnostics_t _diagnostics;         //!< Diagnostic feedback: self monitoring mechanism.

    Result_t _result;                   //!< Result of the processing.



    ///!< Routines that need to be overwritted.
    virtual Status_t run_init() = 0;        //!< Runs only once (at start). Returns _status.
    /* returns: Status_Running or Status_ConcludedWithSuccess or Status_ConcludedWithoutSuccess */

    virtual Status_t run_loop() = 0;        //!< Runs continuously. Returns _status.
    /* returns: Status_Running or Status_ConcludedWithSuccess or Status_ConcludedWithoutSuccess */

    virtual Status_t run_finish() = 0;      //!< Runs only once (at finish). Returns _status.
    /* returns: Status_Running or Status_ConcludedWithSuccess or Status_ConcludedWithoutSuccess */

    virtual void calc_result() = 0;        //!< translates and saves the result of the "run_init"+"run_loop"+"run_finish".
    /* saves: the output to be provided to skill in variable '_result' */

public:
    Action_t();
    Status_t start();                           //!< member that should be execute before the "run()".
    Status_t run();                             //!< Non-blocking! member that should be executed by the Skill for running this action.
    Status_t stop();                            //!< member that should be executed by the Skill for stopping this action.

    void set_source(Goal_t src);         //!< member that should be executed by the Skill for setting the source for the Goal.
    void set_target(const Goal_t &src);         //!< member that should be executed by the Skill for setting the target for the Goal.
    void set_params(const Params_t & src );     //!< member that should be executed by the Skill for setting the parameters that will be used in "run_xpto".

    Status_t get_status();                      //!< member that should be executed by the Skill for getting the status of action.
    Diagnostics_t get_diagnostics();            //!< member that should be executed by the Skill for getting the diagnostics that will be updated in "run_xpto".
    Result_t get_result();                      //!< member that should be executed by the Skill for getting the result that is obtained by the ''calc_result''

    void print_debug();
};


/// Constructor
template<class Params_t, class Goal_t, class Diagnostics_t, class Result_t>
Action_t<Params_t, Goal_t, Diagnostics_t, Result_t>::Action_t()
{
    _status = Status_Idle;
    return;
}

template<class Params_t, class Goal_t, class Diagnostics_t, class Result_t>
Status_t Action_t<Params_t, Goal_t, Diagnostics_t, Result_t>::start()
{
    _status = Status_ReadyToStart;
    return _status;
}


/// Mechanism for running the action.
template<class Params_t, class Goal_t, class Diagnostics_t, class Result_t>
Status_t Action_t<Params_t, Goal_t, Diagnostics_t, Result_t>::run()
{
    Status_t temp;

    switch(_status)
    {

    // run init
    case Status_ReadyToStart:
        temp = run_init();

        if ( temp == Status_Running )
            return Status_Running;

        if ( temp == Status_ConcludedWithoutSuccess )
        {
            _status = Status_Stopped;
            return Status_ConcludedWithoutSuccess;
        }
        if ( temp == Status_ConcludedWithSuccess )
        {
            _status = Status_Running;
            return Status_Running;
        }

        break;

        // run loop
    case Status_Running:

        temp = run_loop();

        if ( temp == Status_Running )
            return Status_Running;
        if ( temp == Status_ConcludedWithoutSuccess )
        {
            _status = Status_Stopped;
            return Status_ConcludedWithoutSuccess;
        }
        if ( temp == Status_ConcludedWithSuccess )
        {
            _status = Status_ConcludedWithSuccess;
            return Status_Running;
        }
        break;

        // run finish
    case Status_ConcludedWithSuccess:

        temp = run_finish();

        if ( temp == Status_Running )
            return Status_Running;
        if ( temp == Status_ConcludedWithoutSuccess )
        {
            _status = Status_Stopped;
            return Status_ConcludedWithoutSuccess;
        }
        if ( temp == Status_ConcludedWithSuccess )
        {
            _status = Status_Idle;
            return Status_ConcludedWithSuccess;

        }
        break;

    default:
        return _status;
    }
}

template<class Params_t, class Goal_t, class Diagnostics_t, class Result_t>
Status_t Action_t<Params_t, Goal_t, Diagnostics_t, Result_t>::stop()
{
    _status = Status_Stopped;
    return _status;
}


template<class Params_t, class Goal_t, class Diagnostics_t, class Result_t>
void Action_t<Params_t, Goal_t, Diagnostics_t, Result_t>::set_source(Goal_t src)
{
    _source = src;
    return;

}

template<class Params_t, class Goal_t, class Diagnostics_t, class Result_t>
void Action_t<Params_t, Goal_t, Diagnostics_t, Result_t>::set_target(const Goal_t &src)
{
    _target = src;
    //std::cout << "linear: " << _target.linear << "\nangular: " << _target.angular << std::endl;
    //_status = Status_ReadyToStart;
    return;
}

template<class Params_t, class Goal_t, class Diagnostics_t, class Result_t>
void Action_t<Params_t, Goal_t, Diagnostics_t, Result_t>::set_params(const Params_t &src)
{
    _params = src;
    return;
}

template<class Params_t, class Goal_t, class Diagnostics_t, class Result_t>
Status_t Action_t<Params_t, Goal_t, Diagnostics_t, Result_t>::get_status()
{
    return _status;
}

template<class Params_t, class Goal_t, class Diagnostics_t, class Result_t>
Diagnostics_t Action_t<Params_t, Goal_t, Diagnostics_t, Result_t>::get_diagnostics()
{
    return _diagnostics;
}

template<class Params_t, class Goal_t, class Diagnostics_t, class Result_t>
Result_t Action_t<Params_t, Goal_t, Diagnostics_t, Result_t>::get_result()
{
    return _result;
}

template<class Params_t, class Goal_t, class Diagnostics_t, class Result_t>
void Action_t<Params_t, Goal_t, Diagnostics_t, Result_t>::print_debug()
{
    std::cout<<" Action_ print()"<<std::endl;
    return;
}


/* ************************************************************************** */
/* ************************************************************************** */
/*    Preparing the routines for the action                                   */
/* ************************************************************************** */
/* ************************************************************************** */
/// To be Overridden
/*
template<class Params_t, class Goal_t, class Diagnostics_t, class Result_t>
Status_t Action_t<Params_t, Goal_t, Diagnostics_t, Result_t>::run_init(){}

/// To be Overridden
template<class Params_t, class Goal_t, class Diagnostics_t, class Result_t>
Status_t Action_t<Params_t, Goal_t, Diagnostics_t, Result_t>::run_loop(){}

/// To be Overridden
template<class Params_t, class Goal_t, class Diagnostics_t, class Result_t>
Status_t Action_t<Params_t, Goal_t, Diagnostics_t, Result_t>::run_finish(){}

/// To be Overridden
template<class Params_t, class Goal_t, class Diagnostics_t, class Result_t>
void Action_t<Params_t, Goal_t, Diagnostics_t, Result_t>::calc_result()
{
    //_result = .... ;
    return;
}
*/

#endif