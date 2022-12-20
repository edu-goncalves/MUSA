#ifndef ROSNODE_GOTO_H
#define ROSNODE_GOTO_H


/*
    Authors: Andry Maykol Pinto.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
    ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
    LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
    CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
    SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
    INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
    STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
    WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <iostream>
#include <string>
#include <vector>

#include <QTimer>
#include <QApplication>

#include <ros/ros.h>
#include <ros/package.h>
#include "std_msgs/Int32.h"
#include "std_srvs/Trigger.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_listener.h"
#include <tf2/convert.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf/tf.h>

#include "task_goto_multiple/vel_cmd.h"

#include "task_goto_seq.hpp"

#include <fstream>
#include <yaml-cpp/yaml.h>

//typedef std::vector<Sk_Goal_t<int,int>> GOAL;

/*!
 * \brief The ROSNode_GoTo class
 *
 * TOPICS:
 * \note ADICIONAR OS SUBSCRIBERS!!!!
 * \note publishes  the "/skill_goto/cmd_vel"             --> geometry_msgs::Twist
 *
 * SERVICES:
 * \note service "/goto/start"                      --> starts or stops gotoxy
 *
 */

//function to add operator >> to the yaml implementation
template<typename T> void operator >> ( const YAML::Node& node, T& i )
{
    i = node.as<T>();
}

class ROSNode_GoTo: public QObject //class that inherits QObject's arguments and methods
{
    Q_OBJECT

private:
    ros::NodeHandle *n_private;     //handler for private parameters
    
    ros::Publisher pub_cmd_vel;     //publisher of velocity commands
    ros::Publisher pub_vlr;

    geometry_msgs::Twist msg_cmd_vel;
    task_goto_multiple::vel_cmd msg_lr;

    tf::TransformListener listener;

    ros::Subscriber sub_odom_source;
    
    
    void cb_receiveOdom_source(const nav_msgs::Odometry::ConstPtr &msg);
    
    ros::ServiceServer service_start;
    bool srv_start_stop(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    bool buildWaypointsFromFile(); // pass array of PointStamped messages by reference

    Task_GoToSequence * task_goto;
    Task_GoToSequence_Params_t _params;
    Task_GoToSequence_Targets_t _target;

    Skill_GoToPose_Goal_t _source;

    QTimer * timer_ROS_Spin;                             //starts in the constructor

public:
    /// Node
    ros::NodeHandle nh;

    /// Constructors.
    ROSNode_GoTo();
    /// Destructors.
    ~ROSNode_GoTo();

    //void ROS_set_skill_params(Skill_GoToPose_Params_t _params);

signals:
    void sig_start_exec(int interval_ms);               //!< starts the continuous loop.
    void sig_stop_exec();

private slots:
    void slot_ros_spin();                               //!< connected to SIGNAL "timer_ROS_Spin" timeout

public slots:
    // ******************************************
    // INPUTS from ACTIONS
    /// SIGNAL: Publishes Linear and Angular velocities.
    void slot_result_vw(float & v, float & w);          //!< SIGNAL "sig__result_vw" from "skill_goto.action_vw".
    // ******************************************
};


#endif // ROSNODE_SKILL_GOTO_H
