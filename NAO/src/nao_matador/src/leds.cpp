
/*
 *   @author: Jaime Andrés González Eelman                                                           
 *   @author: Jesús Andrés Varona                                                                    
 *   @author: Sherif Shousha                                                                         
 *   @author: Wenlan Shen                                                                            
 *                                                                                                   
 *   This module is responsible for controlling the NAO LED.                                                           
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
#include <nao_matador/BlinkAction.h>
#include <nao_matador/Led_msgs.h>

using namespace std;

bool stop_thread=false;
void spinThread()
{
    while(!stop_thread){
        ros::spinOnce();
    }
}

class Nao_control
{

protected:
    //New stuff
    ros::ServiceServer led_serv;

    // ROS node handler
    ros::NodeHandle nh_;

public:

    boost::thread *spin_thread;

    // Create the action client
    actionlib::SimpleActionClient<nao_matador::BlinkAction> my_actionClient;

    // Init the action client: target the /blink action server. Here "true" causes the action client to spin its own thread
    Nao_control(): my_actionClient(nh_, "blink", true){
        led_serv = nh_.advertiseService("/leds", &Nao_control::ledCallback, this);
        ROS_INFO("Service started");

        stop_thread=false;
        spin_thread=new boost::thread(&spinThread);
    }
    ~Nao_control(){
        ROS_WARN_STREAM("Destroy the object.");
        stop_thread=true;
        sleep(1);
        spin_thread->join();
    }

/**
 * Callback function that suscribes to NAO´s LEDs, depending on which action is received, it will return 
 * a different color pattern. 
 */

    bool ledCallback(nao_matador::Led_msgs::Request &req, nao_matador::Led_msgs::Response &res){
        ROS_INFO_STREAM("Got  request for the Leds");
        nao_matador::BlinkGoal blinkGoal;
        std_msgs::ColorRGBA color;
        std::vector<std_msgs::ColorRGBA> colors;
        double time(0.0);
        int it = (int) req.iterations;
        color.r = 0;
        color.g = 0;
        color.b = 0;
        color.a = 0;
        for (int i = 0; i < it; ++i) {
            if(req.action[i].compare("challenge")==0){
                ROS_WARN_STREAM("Challenged the bull");

                color.b = 1;
                color.g = 1;
                color.a = 1;
                time = 1.0;
                blinkGoal.blink_rate_mean = 1.0;
                blinkGoal.blink_rate_sd = 0.1;
            }
            else if(req.action[i].compare("start")==0){
                ROS_WARN_STREAM("Initial gaze");

                color.r = 1;
                color.a = 1;
                time = 2.0;

                blinkGoal.blink_rate_mean = 2.0;
                blinkGoal.blink_rate_sd = 0.1;
            }
            else if(req.action[i].compare("hurra")==0){
                ROS_WARN_STREAM("Succesfull juke");

                color.g = 1;
                color.a = 1;
                time = 1.0;

                blinkGoal.blink_rate_mean = 1.0;
                blinkGoal.blink_rate_sd = 0.1;
            }
            else if(req.action[i].compare("sad")==0){
                ROS_WARN_STREAM("Hit by the bull");

                color.b = 1;
                color.r = 1;
                color.a = 1;
                time = 2.0;

                blinkGoal.blink_rate_mean = 2.0;
                blinkGoal.blink_rate_sd = 0.1;
            }
            else {
                ROS_WARN_STREAM("Cancel goals");
                my_actionClient.cancelAllGoals();
                return false;
            }
            colors.push_back(color);
            blinkGoal.colors = colors;
            blinkGoal.blink_duration = ros::Duration(time);
            my_actionClient.sendGoal(blinkGoal, this->doneBlinkCallback, this->activeBlinkCallback, this->feedbackBlinkCallback);
        }
        return true;
    }

    // Called once when the blink goal completes
    static void doneBlinkCallback(const actionlib::SimpleClientGoalState& state, const nao_matador::BlinkResultConstPtr& result){
        ROS_INFO_STREAM("Finished in state: "<< state.toString().c_str());
        ROS_INFO_STREAM("Finished in state: "<< result);
    }

    // Called once when the blink goal becomes active
    static void activeBlinkCallback(){
        ROS_INFO("Blink goal just went active");
    }

    // Called every time feedback is received for the blink goal
    static void feedbackBlinkCallback(const nao_matador::BlinkFeedbackConstPtr& feedback){
        ROS_INFO_STREAM("Got the following Feedback: "<< feedback->last_color);
    }
};

/**
 * Main function, sets up the node and the communication to other nodes
 * @param argc: input arguments, unused
 * @param argv: input arguments, unused
 * @return int, exists program
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "module_leds");

    // Enter rosspin loop
    Nao_control TermiNAOtor;
    ROS_INFO_STREAM("Started the Client");

    // Waiting for NAO blink action server to start (roslaunch nao_apps leds.launch):
    TermiNAOtor.my_actionClient.waitForServer();
    ROS_INFO_STREAM("Server started");

    // Get into the main loop:
    ros::spin();
    return 0;
}
