/*
 *   @author: Jaime Andrés González Eelman                                                           
 *   @author: Jesús Andrés Varona                                                                    
 *   @author: Sherif Shousha                                                                         
 *   @author: Wenlan Shen                                                                            
 *                                                                                                   
 * 
 * This module is responsible for all the walking movements.
 */                                                           

#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdlib.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include "sensor_msgs/JointState.h"
#include "message_filters/subscriber.h"
#include <string.h>
#include <naoqi_bridge_msgs/JointAnglesWithSpeed.h>
#include <naoqi_bridge_msgs/Bumper.h>
#include <naoqi_bridge_msgs/HeadTouch.h>
#include <naoqi_bridge_msgs/HandTouch.h>
#include <naoqi_bridge_msgs/JointAnglesWithSpeedAction.h>
#include <std_srvs/Empty.h>
#include <boost/algorithm/string.hpp>
#include <boost/date_time.hpp>
#include <naoqi_bridge_msgs/SpeechWithFeedbackActionGoal.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <naoqi_bridge_msgs/BlinkActionGoal.h>
#include <naoqi_bridge_msgs/SetSpeechVocabularyActionGoal.h>
#include <std_msgs/ColorRGBA.h>
#include <naoqi_bridge_msgs/WordRecognized.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Bool.h>
#include "actionlib/client/simple_action_client.h"
#include <nao_matador/Walking_msgs.h>

using namespace std;

bool stop_thread=false;
void spinThread()
{
    while(!stop_thread)
    {
        // sleep(1);
        // ROS_INFO_STREAM("Spinning the thing!!");
        ros::spinOnce();
    }
}



class Nao_control
{

protected:
    //New stuff
    ros::ServiceServer walk_serv;

    // ROS node handler
    ros::NodeHandle nh_;

    // Publisher to nao walking
    ros::Publisher walk_pub;
    ros::ServiceClient stopWalk_srv;

    // Subscriber for foot contact
    ros::Subscriber footContact_sub;

    std_srvs::Empty srv_empty;

public:

    bool walking=false;

    boost::thread *spin_thread;

    // Init the action client: target the /blink action server. Here "true" causes the action client to spin its own thread
    Nao_control(){
        // for walking
        walk_serv = nh_.advertiseService("/walking", &Nao_control::walker, this);
        ROS_INFO("Service started");

        footContact_sub = nh_.subscribe<std_msgs::Bool>("/foot_contact", 1, &Nao_control::footContactCallback, this);
        walk_pub=nh_.advertise<geometry_msgs::Pose2D>("/cmd_pose", 1);
        stopWalk_srv=nh_.serviceClient<std_srvs::Empty>("/stop_walk_srv");

        stop_thread=false;
        spin_thread=new boost::thread(&spinThread);
    }

    ~Nao_control()
    {
        ROS_WARN_STREAM("Destroy the object.");
        stop_thread=true;
        sleep(1);
        spin_thread->join();
    }
/**
 * Callback function that subscribes to NAO´s foot sensors, for safety reasons, if NAO has fallen, it stops walking.
 */
    void footContactCallback(const std_msgs::BoolConstPtr& contact)
    {
        if (!contact->data)
            stopWalk();
    }
    
/**
 * Function that receives the type of movement requested by the state machine and then proceeds to 
 * execute it.
 */
    bool walker(nao_matador::Walking_msgs::Request &req, nao_matador::Walking_msgs::Response &res)
    {
        if(nh_.ok()){
            int it = (int) req.iterations;
            for (int i = 0; i < it; ++i) {
                geometry_msgs::Pose2D msg;
                msg.x = 0;
                msg.y = 0;
                msg.theta = 0;
                if(req.action[i].compare("turn")==0){
                    walking = true;
                    msg.theta = req.target[i];
                    walk_pub.publish(msg);
                    ROS_WARN_STREAM("ROTATING: " << req.target[i]*180/M_PI << " Degrees");
                    continue;
                }
                else if(req.action[i].compare("side")==0){
                    walking = true;
                    msg.y = req.target[i];
                    walk_pub.publish(msg);
                    ROS_WARN_STREAM("MOVING SIDEWAYS: " << req.target[i] << " m");
                    continue;
                }
                else if(req.action[i].compare("forward")==0){
                    walking = true;
                    msg.x = req.target[i];
                    walk_pub.publish(msg);
                    ROS_WARN_STREAM("MOVING FORWARD: " << req.target[i] << " m");
                    continue;
                }
                return res.walking = false;
            }
            return res.walking = true;
        }
        else
            return res.walking = false;
    }

/**
 * Function to stop walking
 */

    void stopWalk()
    {
        stopWalk_srv.call(srv_empty);
        walking = false;
        ROS_WARN("Stop walking");
    }

};
int main(int argc, char** argv)
{
    ros::init(argc, argv, "module_walking");

    // Enter rosspin loop
    Nao_control TermiNAOtor;
    ROS_INFO_STREAM("Started the Client");

    // Get into the main loop:
   // ros::Rate rate_sleep(10);
    ros::spin();
    return 0;
}
