#ifndef MISSION_GOTO_HPP
#define MISSION_GOTO_HPP

#include "task_t.hpp"
#include "skill_goto.hpp"

#include <iostream>
#include <string.h>

//#include <QWidget>
#include <QApplication>
#include <QTimer>
#include <QtGui>

class QObject;

struct Task_GoToSequence_Targets_t
{
    std::vector<Skill_GoToPose_Goal_t> _waypoints;
};

struct Task_GoToSequence_Params_t
{
    //PARAMETERS Task
    //....
    Skill_GoToPose_Params_t params_skill_goto;
};

struct Task_GoToSequence_Diagnostics_t
{
    std::string message;
    int mode;      //-1 error
};

struct Task_GoToSequence_Result_t
{
    bool isValid;
    Task_GoToSequence_Targets_t pose_achieved;  // pose that was reached by the controller
    Task_GoToSequence_Targets_t pose_error;     // error between actual and desired pose
};


class Task_GoToSequence: public QObject, public Task_t<Task_GoToSequence_Params_t, Task_GoToSequence_Targets_t, Task_GoToSequence_Diagnostics_t,Task_GoToSequence_Result_t>
{
    Q_OBJECT

private:
    T_Status_t run_init();        //!< Runs only once (at start). Returns _status.
    /* returns: Status_Running or Status_ConcludedWithSuccess or Status_ConcludedWithoutSuccess */

    T_Status_t run_loop();        //!< Runs continuously. Returns _status.
    /* returns: Status_Running or Status_ConcludedWithSuccess or Status_ConcludedWithoutSuccess */

    T_Status_t run_finish();      //!< Runs only once (at finish). Returns _status.
    /* returns: Status_Running or Status_ConcludedWithSuccess or Status_ConcludedWithoutSuccess */

    Sk_Status_t _skill_exec(); //!< executes all actions at once.


    // User-defined functions/procedures
    QTimer  * timer_run;

    //! Current pose target from the array of targets
    size_t _Stages_GoToSequence;
    
    //long _time_loops_elapsed; // user-defined variable: to wait between stages. .

public:
    Task_GoToSequence();
    ~Task_GoToSequence();

    void set_timers();

    //user defined stuffs
    //...
    //! SKILL:: GoTo
    Skill_GoToPose * skill_goto;


private slots:
    void slot_run();                                     //!< Slot for the SIGNAL (timer.timeout).
    //void slot_timeout_ME();

public slots:
    //void slot_onDemand();                                //!< Slot for the SIGNAl (to be defined)

    void slot_start_exec(int interval_ms);               //!< Slot starts the continuous loop.
    void slot_stop_exec();                               //!< Slot stops the continuous loop.

signals:
    /// SIGNAL: Feedback of the SKILL.
    void task_sig_feedback(T_Status_t & feedback);

    void sig_result_vw(float & v, float & w);
};


#endif
