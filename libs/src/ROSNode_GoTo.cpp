#include "ROSNode_GoTo.hpp"


//constructor
ROSNode_GoTo::ROSNode_GoTo()
{  
    n_private                   = new ros::NodeHandle("~");

    //topic where to publish vel commands
    std::string cmd_vel_topic;
    n_private->param<std::string>("cmd_vel_topic", cmd_vel_topic, "/auto");
    
    //
    pub_cmd_vel                 = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1);
    pub_vlr                     = nh.advertise<task_goto_multiple::vel_cmd>("vel_cmd_lr", 1);

    // Subscribes the topic.
    std::string odom_topic;

    n_private->param<std::string>("odom_topic", odom_topic, "/odometry/filtered");
    

    sub_odom_source            = nh.subscribe(odom_topic, 1,  &ROSNode_GoTo::cb_receiveOdom_source, this);

    // Service to start/stop the skill.
    service_start               =  nh.advertiseService("/goto/start", &ROSNode_GoTo::srv_start_stop, this);

    // timer to see if there is messages received from ROS
    timer_ROS_Spin = new QTimer();
    timer_ROS_Spin->setInterval(50);//milliseconds

    /// PARAMETERS Task_goto
    /// ....

    /// PARAMETERS Skill_goto

    n_private->param<float>("EPS_yaw", _params.params_skill_goto.EPS_pose._orientation.yaw, M_PI / 20);
    n_private->param<float>("EPS_x", _params.params_skill_goto.EPS_pose._position.x, 1.0);
    n_private->param<float>("EPS_y", _params.params_skill_goto.EPS_pose._position.y, 1.0);
    n_private->param<float>("W_MAX", _params.params_skill_goto.W_MAX, 0.753);
    n_private->param<float>("W_MIN", _params.params_skill_goto.W_MIN, 0.2);
    n_private->param<float>("V_MAX", _params.params_skill_goto.V_MAX, 2.0);
    n_private->param<float>("V_MIN", _params.params_skill_goto.V_MIN, 0.25);
    n_private->param<float>("thrs_time_start_move", _params.params_skill_goto.thrs_time_start_move, 100);
    n_private->param<float>("D_Finish", _params.params_skill_goto.D_Finish, 1.0);
    n_private->param<float>("W_active", _params.params_skill_goto.W_active, 1.0);
    n_private->param<int>("Move_End_Time", _params.params_skill_goto.Move_End_Time, 60);

    

    ///-> setting parameters for ACTIONS

    _params.params_skill_goto.params_action_vw.gain_angular        = 0.5; //user defined - to be adjusted based on thrusters
    _params.params_skill_goto.params_action_vw.gain_linear         = 1.0; //user defined - to be adjusted based on thrusters
    _params.params_skill_goto.params_action_vw.max_angular_acc     = 1.2; //0.6
    _params.params_skill_goto.params_action_vw.max_linear_acc      = 2.0; //1
    _params.params_skill_goto.params_action_vw.delta_time          = 0.04;  //seconds
    _params.params_skill_goto.params_action_vw.half_baseline       = 0.35;  //m
    _params.params_skill_goto.params_action_vw.max_angular_speed   = 1.0;
    _params.params_skill_goto.params_action_vw.max_linear_speed    = 1.0;
    _params.params_skill_goto.params_action_vw.linear_scaling      = 1.0;
    _params.params_skill_goto.params_action_vw.angular_scaling     = 1.0;
    _params.params_skill_goto.params_action_vw.rpm                 = 1000;

    ///-> other action...
    //...

    task_goto = new Task_GoToSequence();
    task_goto->skill_goto->action_vw->set_params(_params.params_skill_goto.params_action_vw);
    task_goto->skill_goto->set_params(_params.params_skill_goto);
    task_goto->set_params(_params);
    task_goto->skill_goto->set_timers();
  
    
    //QObject::connect(const QObject *sender, const char *signal, const QObject *receiver, const char *method, Qt::ConnectionType type = Qt::AutoConnection)
    QObject::connect(this->task_goto, SIGNAL(sig_result_vw(float&,float&)), this, SLOT(slot_result_vw(float&,float&)));
    
    QObject::connect(this, SIGNAL(sig_start_exec(int)), this->task_goto, SLOT(slot_start_exec(int)));

    QObject::connect(this, SIGNAL(sig_stop_exec()), this->task_goto, SLOT(slot_stop_exec()));

    QObject::connect(this->timer_ROS_Spin, SIGNAL(timeout()), this, SLOT(slot_ros_spin()));
    
    timer_ROS_Spin->start();
    return;
}



ROSNode_GoTo::~ROSNode_GoTo()
{
    delete task_goto;
    delete timer_ROS_Spin;
}

void ROSNode_GoTo::slot_ros_spin()
{
    ros::spinOnce();
    return;
}


void ROSNode_GoTo::cb_receiveOdom_source(const nav_msgs::Odometry::ConstPtr &msg)
{
    //Pose
    tf::Quaternion q(
                msg->pose.pose.orientation.x,
                msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z,
                msg->pose.pose.orientation.w);

    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    _source._orientation.roll  = normalize_angle(roll);
    _source._orientation.pitch = normalize_angle(pitch);
    _source._orientation.yaw   = normalize_angle(yaw);
    _source._position.x = msg->pose.pose.position.x;
    _source._position.y = msg->pose.pose.position.y;
    _source._position.z = msg->pose.pose.position.z;

    task_goto->skill_goto->set_source(_source);

    //Twist
    Act_SetVW_Goal_t _current_vel;
    _current_vel.linear  = msg->twist.twist.linear.x;
    _current_vel.angular = msg->twist.twist.angular.z;
    
    task_goto->skill_goto->action_vw->set_source(_current_vel);

    return;
}


bool ROSNode_GoTo::buildWaypointsFromFile() 
{
    Task_GoToSequence_Targets_t waypoints;

    std::string waypoints_filename = "cfg/waypoints.yaml";
    std::string waypoints_path_filename;

    // get the package path
    std::string pkg_path = ros::package::getPath("task_goto_multiple"); 
    waypoints_path_filename = pkg_path + "/" + waypoints_filename;

    try
    {
        std::ifstream ifs(waypoints_path_filename.c_str(), std::ifstream::in);
        if (!ifs.good())
        {
            ROS_FATAL( "buildWaypointsFromFile could not open file" );// if it cannot open the file check path, package name
            return false;
        }
    
        // yaml node
        YAML::Node yaml_node;
        // load file
        yaml_node = YAML::Load(ifs);
        // read data
        const YAML::Node &wp_node_tmp = yaml_node[ "waypoints" ];
        const YAML::Node *wp_node = wp_node_tmp ? &wp_node_tmp : NULL;

        if (wp_node != NULL)
        {
            for (int i = 0; i < wp_node->size(); i++)
            {
                tf::StampedTransform transform;
                geometry_msgs::PoseStamped aux; //to store from file
                float th;

                (*wp_node)[i]["point"]["frame"] >> aux.header.frame_id;
                (*wp_node)[i]["point"]["x"] >> aux.pose.position.x;
                (*wp_node)[i]["point"]["y"] >> aux.pose.position.y;
                aux.pose.position.z = 0;             //initialize to zero
                (*wp_node)[i]["point"]["th"] >> th;  

                double roll, pitch, yaw;
                roll = pitch = 0;
                th = (double) th;

                //if coordinate pose is expressed in the UTM frame -> transform to world frame
                if(aux.header.frame_id.find("utm") != std::string::npos)
                {
                    //convert RPY to quaternion
                    tf2::Quaternion q_aux;
                    
                    q_aux.setRPY((tf2Scalar)0, (tf2Scalar)0, (tf2Scalar)th);
                    aux.pose.orientation = tf2::toMsg(q_aux);
                    
                    try
                    {
                        listener.lookupTransform("/world", "/utm", ros::Time(0), transform);
                        listener.transformPose("/world", aux, aux);

                    }
                    catch(tf::TransformException ex)
                    {
                        ROS_ERROR("%s",ex.what());
                        ros::Duration(1.0).sleep();
                        return false;
                    }

                    //convert quaternion to RPY
                    tf2::fromMsg(aux.pose.orientation, q_aux);
                    tf2::Matrix3x3 m_aux(q_aux);

                    m_aux.getRPY(roll, pitch, yaw);
                }
                //aux variable to store position
                Skill_GoToPose_Goal_t aux_waypoints;

                aux_waypoints._position.x = aux.pose.position.x;
                aux_waypoints._position.y = aux.pose.position.y;
                aux_waypoints._position.z = aux.pose.position.z;

                aux_waypoints._orientation.roll = roll;
                aux_waypoints._orientation.pitch = pitch;
                aux_waypoints._orientation.yaw = yaw;

                waypoints._waypoints.push_back(aux_waypoints);
            }
        }

        else
        {
            return false;
        }
    }
    catch (YAML::ParserException &e)
    {
        return false;
    }
    catch (YAML::RepresentationException &e)
    {
        return false;
    }

    //set task targets
    task_goto->set_target(waypoints);

    return true;
}


///*
/// Service that starts or stops the skill
bool ROSNode_GoTo::srv_start_stop(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    std::cout << "service call" << std::endl;

    if((task_goto->get_status() != T_Status_Idle) && (task_goto->get_status() != T_Status_Stopped) && (task_goto->get_status() != T_Status_Error))
    {
        emit sig_stop_exec();
        res.message = "ROSNode_GoTo:: Service:: stop";
    }
    else
    {
        bool built = buildWaypointsFromFile();

        if ( !built )
        {
            ROS_FATAL( "building waypoints from a file failed" );
            return built;
        }

        emit sig_start_exec(_params.params_skill_goto.params_action_vw.delta_time*100);
        res.message = "ROSNode_GoTo:: Service:: start";
    }
    res.success = 1;

    return true;
}


///*
/// Slot that publishes the CMD_VEL
void ROSNode_GoTo::slot_result_vw(float &v, float &w)
{
    msg_cmd_vel.linear.x  = v;
    msg_cmd_vel.angular.z = w;
   
    pub_cmd_vel.publish(msg_cmd_vel);

    //publish v_left and v_right from kinematics
    Act_SetVW_Result_t aux_result = task_goto->skill_goto->action_vw->get_result();
    
    msg_lr.left_cmd = aux_result.V_left;
    msg_lr.right_cmd = aux_result.V_right;

    pub_vlr.publish(msg_lr);

    return;
}

