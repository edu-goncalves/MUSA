#include <QtWidgets/QApplication>
#include <iostream>
#include <unistd.h>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include "ROSNode_GoTo.hpp"


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "task_goto_multiple");
    QCoreApplication app(argc, argv);

    ROSNode_GoTo * task = new ROSNode_GoTo();

    ros::spinOnce();
    return app.exec();
}