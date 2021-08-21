/*
 *   @author: Jaime Andrés González Eelman                                                           
 *   @author: Jesús Andrés Varona                                                                    
 *   @author: Sherif Shousha                                                                         
 *   @author: Wenlan Shen                                                                            
 *                                                                                                   
 *   This module is responsible for all of the information regarding the bumpers. It publishes the   
 *   topic /hit to detect bull collisions.                                                           
*/

#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdlib.h>
#include <ros/ros.h>
#include "message_filters/subscriber.h"
#include <naoqi_bridge_msgs/Bumper.h>
#include <std_msgs/Bool.h>



using namespace std;

ros::Subscriber bumper_sub;
ros::Publisher hit_pub;

/**
 * Callback function that subscribes to the bumpers nao_app and publishes the state of the bumper
 * @param bumperState: bumper object from wich the pressed state is compared
 */
void bumperCallback(const naoqi_bridge_msgs::Bumper::ConstPtr& bumperState) {
    ROS_INFO_STREAM("Got something");
    std_msgs::Bool msg;
    if (bumperState->state == bumperState->statePressed){
        ROS_WARN_STREAM("Pressed a bumper");
        msg.data = true;
    }
    else
        msg.data = false;
    hit_pub.publish(msg);
}

/**
 * Main function, sets up the node and the communication to other nodes
 * @param argc: input arguments, unused
 * @param argv: input arguments, unused
 * @return int, exists program
 */
int main(int argc, char** argv){
    ros::init(argc, argv, "module_bumpers");
    ros::NodeHandle nh;

    bumper_sub=nh.subscribe("/bumper",1, bumperCallback);
    hit_pub = nh.advertise<std_msgs::Bool>("/hit", 1);

    ros::spin();
    return 0;
}