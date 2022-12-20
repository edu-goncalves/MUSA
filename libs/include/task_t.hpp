#ifndef MISSION_T_HPP
#define MISSION_T_HPP

#include <iostream>


template<typename Value, typename Data> struct T_Goal_t
{
    Value value = 0;
    Data data;
};


//! T_Params_t template for providing parameters to the Task.
/*!
 * Data (can be a composed variable, eg. a structure)
 *
 */
template<typename Data> struct T_Params_t
{
    Data data;
};



//! T_Result_t template for reporting the result of the task after conclusion.
/*!
 * Data (can be a composed variable, eg. a structure)
 *
 */
template<typename Data> struct T_Result_t
{
    Data data;
};



//! Diagnostic_t template for reporting the "fault".
/*!
 * Data (can be a composed variable, eg. a structure)
 *
 */
template<typename Data> struct T_Diagnostics_t
{
    Data data;
};




//! Modes for the "health"/status of the task for monitoring
enum T_Status_t
{
    T_Status_Idle = 0,
    T_Status_ReadyToStart = 1,
    T_Status_Running = 2,
    T_Status_Paused = 3,
    T_Status_ConcludedWithSuccess = 4,
    T_Status_ConcludedWithoutSuccess = 5,
    T_Status_Stopped = 6,
    T_Status_Aborting = 7,
    T_Status_Error = 8
};


//! Template class for "Task"
/*!
* typedef T_Goal_t<int,int> GOAL;
* ...
* Task_t<GOAL, GOAL, GOAL, GOAL>  task;
* action.print();
*
*/
template <class T_Params_t, class T_Goal_t, class T_Diagnostics_t, class T_Result_t>
class Task_t
{

protected:
    T_Goal_t _target;                     //!< Desired target for completion.

    T_Params_t _params;                   //!< Parameters required by the action.
    T_Status_t _status;                   //!< Current status of the action.
    T_Diagnostics_t _diagnostics;         //!< Diagnostic feedback: self monitoring mechanism.

    T_Result_t _result;                   //!< Result of the processing.

    T_Status_t run();                             //!< Non-blocking! member that should be executed by the Skill for running this action.
    T_Status_t stop();                            //!< member that should be executed by the Skill for stopping this action.
    T_Status_t start();


    ///!< Routines that need to be overwritted.
    virtual T_Status_t run_init() = 0;        //!< Runs only once (at start). Returns _status.
    /* returns: Status_Running or Status_ConcludedWithSuccess or Status_ConcludedWithoutSuccess */

    virtual T_Status_t run_loop() = 0;        //!< Runs continuously. Returns _status.
    /* returns: Status_Running or Status_ConcludedWithSuccess or Status_ConcludedWithoutSuccess */

    virtual T_Status_t run_finish() = 0;      //!< Runs only once (at finish). Returns _status.
    /* returns: Status_Running or Status_ConcludedWithSuccess or Status_ConcludedWithoutSuccess */

    //to be implemented
    /*
    virtual void calc_result();        //!< translates and saves the result of the "run_init"+"run_loop"+"run_finish".
    // saves: the output to be provided to task in variable '_result'
    */

public:
    Task_t();

    void set_source(const T_Goal_t &src);         //!< member that should be executed by the Task for setting the source for the Goal.
    void set_target(const T_Goal_t &src);         //!< member that should be executed by the Task for setting the target for the Goal.
    void set_params(const T_Params_t & src );     //!< member that should be executed by the Task for setting the parameters that will be used in "run_xpto".

    T_Params_t get_params();                      //!< member that should be executed by the Task for getting the parameters.
    T_Status_t get_status();                      //!< member that should be executed by the Task for getting the status of action.
    T_Diagnostics_t get_diagnostics();            //!< member that should be executed by the Task for getting the diagnostics that will be updated in "run_xpto".
    T_Result_t get_result();                      //!< member that should be executed by the Task for getting the result that is obtained by the ''calc_result''

    void print_debug();
};



/// Constructor
template<class T_Params_t, class T_Goal_t, class T_Diagnostics_t, class T_Result_t>
Task_t<T_Params_t, T_Goal_t, T_Diagnostics_t, T_Result_t>::Task_t()
{
    _status = T_Status_Idle;
    return;
}


/// Mechanism for running the task.
template<class T_Params_t, class T_Goal_t, class T_Diagnostics_t, class T_Result_t>
T_Status_t Task_t<T_Params_t, T_Goal_t, T_Diagnostics_t, T_Result_t>::run()
{
    T_Status_t temp;

    switch(_status)
    {

    // run init
    case T_Status_ReadyToStart:

        temp = run_init();

        if ( temp == T_Status_Running )
            return T_Status_Running;
        if ( temp == T_Status_ConcludedWithoutSuccess )
        {
            _status = T_Status_Stopped;
            return T_Status_ConcludedWithoutSuccess;
        }
        if ( temp == T_Status_ConcludedWithSuccess )
        {
            _status = T_Status_Running;
            return T_Status_Running;
        }

        break;

        // run loop
    case T_Status_Running:

        temp = run_loop();

        if ( temp == T_Status_ConcludedWithSuccess )
        {
            _status = T_Status_ConcludedWithSuccess;
            return T_Status_Running;
        }

        if ( temp == T_Status_Running )
            return T_Status_Running;
        
        if ( temp == T_Status_ConcludedWithoutSuccess )
        {
            _status = T_Status_Stopped;
            return T_Status_ConcludedWithoutSuccess;
        }
        
        break;

        // run finish
    case T_Status_ConcludedWithSuccess:
        temp = run_finish();

        if ( temp == T_Status_Running )
        {
            return T_Status_Running;
        }
        if ( temp == T_Status_ConcludedWithoutSuccess )
        {
            _status = T_Status_Stopped;
            return T_Status_ConcludedWithoutSuccess;
        }
        if ( temp == T_Status_ConcludedWithSuccess )
        {
            _status = T_Status_Idle;
            return T_Status_ConcludedWithSuccess;
        }
        break;

    default:
        return _status;
    }

}

template<class T_Params_t, class T_Goal_t, class T_Diagnostics_t, class T_Result_t>
T_Status_t Task_t<T_Params_t, T_Goal_t, T_Diagnostics_t, T_Result_t>::stop()
{
    _status = T_Status_Stopped;
    return _status;
}

template<class T_Params_t, class T_Goal_t, class T_Diagnostics_t, class T_Result_t>
T_Status_t Task_t<T_Params_t, T_Goal_t, T_Diagnostics_t, T_Result_t>::start()
{
    _status = T_Status_ReadyToStart;
    return _status;
}

template<class T_Params_t, class T_Goal_t, class T_Diagnostics_t, class T_Result_t>
void Task_t<T_Params_t, T_Goal_t, T_Diagnostics_t, T_Result_t>::set_target(const T_Goal_t &src)
{
    _target = src;
    return;
}

template<class T_Params_t, class T_Goal_t, class T_Diagnostics_t, class T_Result_t>
void Task_t<T_Params_t, T_Goal_t, T_Diagnostics_t, T_Result_t>::set_params(const T_Params_t &src)
{
    _params = src;
    return;
}


template<class T_Params_t, class T_Goal_t, class T_Diagnostics_t, class T_Result_t>
T_Params_t Task_t<T_Params_t, T_Goal_t, T_Diagnostics_t, T_Result_t>::get_params()
{
    return _params;
}

template<class T_Params_t, class T_Goal_t, class T_Diagnostics_t, class T_Result_t>
T_Status_t Task_t<T_Params_t, T_Goal_t, T_Diagnostics_t, T_Result_t>::get_status()
{
    return _status;
}

template<class T_Params_t, class T_Goal_t, class T_Diagnostics_t, class T_Result_t>
T_Diagnostics_t Task_t<T_Params_t, T_Goal_t, T_Diagnostics_t, T_Result_t>::get_diagnostics()
{
    return _diagnostics;
}

template<class T_Params_t, class T_Goal_t, class T_Diagnostics_t, class T_Result_t>
T_Result_t Task_t<T_Params_t, T_Goal_t, T_Diagnostics_t, T_Result_t>::get_result()
{
    return _result;
}

template<class T_Params_t, class T_Goal_t, class T_Diagnostics_t, class T_Result_t>
void Task_t<T_Params_t, T_Goal_t, T_Diagnostics_t, T_Result_t>::print_debug()
{
    std::cout<<" Task_ print()"<<std::endl;
    return;
}


/* ************************************************************************** */
/* ************************************************************************** */
/*    Preparing the routines for the task                                  */
/* ************************************************************************** */
/* ************************************************************************** */
/// To be Overridden
/*
template<class T_Params_t, class T_Goal_t, class T_Diagnostics_t, class T_Result_t>
T_Status_t Task_t<T_Params_t, T_Goal_t, T_Diagnostics_t, T_Result_t>::run_init(){}

/// To be Overridden
template<class T_Params_t, class T_Goal_t, class T_Diagnostics_t, class T_Result_t>
T_Status_t Task_t<T_Params_t, T_Goal_t, T_Diagnostics_t, T_Result_t>::run_loop(){}

/// To be Overridden
template<class T_Params_t, class T_Goal_t, class T_Diagnostics_t, class T_Result_t>
T_Status_t Task_t<T_Params_t, T_Goal_t, T_Diagnostics_t, T_Result_t>::run_finish(){}

*/

/// To be Overridden
/*
template<class T_Params_t, class T_Goal_t, class T_Diagnostics_t, class T_Result_t>
void Task_t<T_Params_t, T_Goal_t, T_Diagnostics_t, T_Result_t>::calc_result(){}
*/

#endif


