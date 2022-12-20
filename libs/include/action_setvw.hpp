#ifndef ACTION_SW_HPP
#define ACTION_SW_HPP

#include "action_t.hpp"
#include <iostream>
#include <string.h>


struct Act_SetVW_Goal_t
{
    float linear;
    float angular;
};


struct Act_SetVW_Params_t
{
    float gain_linear;
    float gain_angular;
    float max_linear_acc;
    float max_angular_acc;
    float max_linear_speed;
    float max_angular_speed;
    float delta_time;
    float half_baseline;
    float linear_scaling;
    float angular_scaling;
    float rpm;
};

struct Act_SetVW_Diagnostics_t
{
    std::string message;
    int mode;
};

struct Act_SetVW_Result_t
{
    float V_left;
    float V_right;
};




class Act_SetVW: public Action_t<Act_SetVW_Params_t, Act_SetVW_Goal_t, Act_SetVW_Diagnostics_t,Act_SetVW_Result_t>
{
private:
    Status_t run_init();        //!< Runs only once (at start). Returns _status.
    /* returns: Status_Running or Status_ConcludedWithSuccess or Status_ConcludedWithoutSuccess */

    Status_t run_loop();        //!< Runs continuously. Returns _status.
    /* returns: Status_Running or Status_ConcludedWithSuccess or Status_ConcludedWithoutSuccess */

    Status_t run_finish();      //!< Runs only once (at finish). Returns _status.
    /* returns: Status_Running or Status_ConcludedWithSuccess or Status_ConcludedWithoutSuccess */

    void calc_result();         //!< translates and saves the result of the "run_init"+"run_loop"+"run_finish".
    /* saves: the output to be provided to skill in variable '_result' */

    // User-defined functions/procedures
    void ASV_kinematics(const Act_SetVW_Goal_t & desired_velocity, float &V_left, float &V_right);
    double limitVel(const double &vel_, const double &max_vel_);

    template <typename T> inline T sgn(T val){ return (T(0) < val) - (val < T(0));}
 
public:
    Act_SetVW();

    /// *****************************************
    /// OUTPUT VARIABLES :: user defined stuffs.
    /// cmd_vel
    Act_SetVW_Goal_t desired_velocity;
    /// Thrusters vel
    float V_right, V_left;
    /// *****************************************

};


#endif
/* //Example of usage

typedef Goal_t<int,int> GOAL;


...

    // Define parameters for Action SetVW
    Act_SetVW_Params_t _params;
    _params.gain_angular        = 0.5; //user defined - to be adjusted based on thrusters
    _params.gain_linear         = 1.0; //user defined - to be adjusted based on thrusters
    _params.max_angular_acc     = 0.6;
    _params.max_linear_acc      = 1.0;
    _params.delta_time          = 0.04;  //seconds
    _params.half_baseline       = 0.35;  //m
    _params.max_angular         = 3.1415;
    _params.max_linear          = 2.0;


    Act_SetVW_Goal_t _source, _target;
    // current velocity (measured)
    _source.angular             = 0.0; //rad/s
    _source.linear              = 0.0; //m/s

    // target velocity
    _target.angular             = 0.8; //rad/s
    _target.linear              = 1.5; //m/s


    /// Create action
    ///
    ///
    Act_SetVW action_setvw_asv;
    action_setvw_asv.set_params(_params);
    action_setvw_asv.set_source(_source);
    action_setvw_asv.set_target(_target);




    //SIMULATE @1Hz

    /// Run Action SetVW
    for (int i = 0; i < 30; i++)
    {
      action_setvw_asv.run();
      sleep(1);
      action_setvw_asv.set_source(action_setvw_asv.desired_velocity);
    }

    std::cout<<" **************"<<std::endl;
    std::cout<<" **************"<<std::endl;
    std::cout<<" **************"<<std::endl;
    action_setvw_asv.set_target(_target);

    for (int i = 0; i < 15; i++)
    {
      action_setvw_asv.run();
      sleep(1);
      action_setvw_asv.set_source(action_setvw_asv.desired_velocity);
    }


    std::cout<<" **************"<<std::endl;
    std::cout<<" **************"<<std::endl;
    std::cout<<" **************"<<std::endl;
    // target velocity
    _target.angular             = 0.0; //rad/s
    _target.linear              = -0.5; //m/s
    action_setvw_asv.set_target(_target);

    for (int i = 0; i < 55; i++)
    {
      action_setvw_asv.run();
      sleep(1);
      action_setvw_asv.set_source(action_setvw_asv.desired_velocity);
    }

    */





