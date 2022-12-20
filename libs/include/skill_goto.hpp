#ifndef SKILL_GOTO_HPP
#define SKILL_GOTO_HPP

#include "skill_t.hpp"

#include <iostream>
#include <string.h>

//#include <QWidget>
#include <QApplication>
#include <QTimer>
#include <QtGui>

#include "action_setvw.hpp"

class QObject;

struct orientation {
    float roll;
    float pitch;
    float yaw;
};
struct position {
    float x;
    float y;
    float z;
};

struct Skill_GoToPose_Goal_t
{
    orientation _orientation;
    position _position;           // target pose
    inline Skill_GoToPose_Goal_t operator-=(Skill_GoToPose_Goal_t a)
    {
        this->_orientation.roll  -= a._orientation.roll;
        this->_orientation.pitch -= a._orientation.pitch;
        this->_orientation.yaw   -= a._orientation.yaw;
        this->_position.x -= a._position.x;
        this->_position.y -= a._position.y;
        this->_position.z -= a._position.z;
    }
    inline Skill_GoToPose_Goal_t operator-(Skill_GoToPose_Goal_t a)
    {
        Skill_GoToPose_Goal_t temp;
        temp._orientation.roll = this->_orientation.roll  - a._orientation.roll;
        temp._orientation.pitch = this->_orientation.pitch - a._orientation.pitch;
        temp._orientation.yaw = this->_orientation.yaw   - a._orientation.yaw;
        temp._position.x = this->_position.x - a._position.x;
        temp._position.y = this->_position.y - a._position.y;
        temp._position.z = this->_position.z - a._position.z;
        return temp;
    }
    inline Skill_GoToPose_Goal_t operator+=(Skill_GoToPose_Goal_t a)
    {
        this->_orientation.roll  += a._orientation.roll;
        this->_orientation.pitch += a._orientation.pitch;
        this->_orientation.yaw   += a._orientation.yaw;
        this->_position.x += a._position.x;
        this->_position.y += a._position.y;
        this->_position.z += a._position.z;
    }
    inline Skill_GoToPose_Goal_t operator+(Skill_GoToPose_Goal_t a)
    {
        Skill_GoToPose_Goal_t temp;
        temp._orientation.roll = this->_orientation.roll  + a._orientation.roll;
        temp._orientation.pitch = this->_orientation.pitch + a._orientation.pitch;
        temp._orientation.yaw = this->_orientation.yaw   + a._orientation.yaw;
        temp._position.x = this->_position.x + a._position.x;
        temp._position.y = this->_position.y + a._position.y;
        temp._position.z = this->_position.z + a._position.z;
        return temp;
    }
};

struct Skill_GoToPose_Params_t
{
     Skill_GoToPose_Goal_t EPS_pose;      // max error allowed:: _params.EPS_pose._position.x && _params.EPS_pose._orientation.yaw
     float W_MAX;                         //[rad/s]
     float W_MIN;                         //[rad/s]
     float V_MAX;                         //[m/s]
     float V_MIN;                         //[m/s]
     float thrs_time_start_move;          //[cicle counts]
     float D_Finish;                      //approach cicle radius for reducing velocity[m]
     float W_active;
     Act_SetVW_Params_t params_action_vw; // parameters for setting action SetVW
     int   Move_End_Time;
};

struct Skill_GoToPose_Diagnostics_t
{
    std::string message;
    int mode;      //-1 error
};

struct Skill_GoToPose_Result_t
{
    bool isValid;
    Skill_GoToPose_Goal_t pose_achieved;  // pose that was reached by the controller
    Skill_GoToPose_Goal_t pose_error;     // current error between actual and desired pose
};



/// Stages for Skill's work
enum _Stages_GoToPose
{
    _Stage_Idle = -1,
    _Stage_Init = 0,
    _Stage_Rotate_Target = 1,
    _Stage_Move_Start = 2,
    _Stage_Move_Cruse = 3,
    _Stage_Move_End = 4,
    _Stage_Completed = 5
};



class Skill_GoToPose: public QObject, public Skill_t<Skill_GoToPose_Params_t, Skill_GoToPose_Goal_t, Skill_GoToPose_Diagnostics_t,Skill_GoToPose_Result_t>
{
    Q_OBJECT

private:
    Sk_Status_t run_init();        //!< Runs only once (at start). Returns _status.
    /* returns: Status_Running or Status_ConcludedWithSuccess or Status_ConcludedWithoutSuccess */

    Sk_Status_t run_loop();        //!< Runs continuously. Returns _status.
    /* returns: Status_Running or Status_ConcludedWithSuccess or Status_ConcludedWithoutSuccess */

    Sk_Status_t run_finish();      //!< Runs only once (at finish). Returns _status.
    /* returns: Status_Running or Status_ConcludedWithSuccess or Status_ConcludedWithoutSuccess */

    void calc_result();         //!< translates and saves the result of the "run_init"+"run_loop"+"run_finish".
    /* saves: the output to be provided to skill in variable '_result' */

    void _actions_exec(); //!< executes all actions at once.


    // User-defined functions/procedures
    QTimer  * timer_run;
    QTimer  *timer_move_end;

    //! Current stage of the work that needs to be done by the skill.
    _Stages_GoToPose state_machine;
    long _time_loops_elapsed; // user-defined variable: to wait between stages. .

public:
    Skill_GoToPose();
    ~Skill_GoToPose();

    void set_timers();

    //user defined stuffs
    //...
    //! ACTION:: setVW
    Act_SetVW * action_vw;


private slots:
    //void slot_run();                                     //!< Slot for the SIGNAL (timer.timeout).
    void slot_timeout_ME();

/*
public slots:
    void slot_onDemand();                                //!< Slot for the SIGNAl (to be defined)

    void slot_start_exec(int interval_ms);               //!< Slot starts the continuous loop.
    void slot_stop_exec();                               //!< Slot stops the continuous loop.
*/

signals:
    /// SIGNAL: Feedback of the SKILL.
    //void sig_feedback(Sk_Status_t & feedback);

    // ******************************************
    // OUTPUTs from ACTIONS
    /// SIGNAL: Linear and Angular velocities.
    //void sig_result_vw(float & v, float & w);

    /// SIGNAL: Vleft and Vright velocities.
    //void sig_result_v1v2(float &v1, float &v2);
    // ******************************************

};

float euclidean_distance (const position &a, const position &b);
float euclidean_distance (const position &a);
float normalize_angle(float ang);

#endif
