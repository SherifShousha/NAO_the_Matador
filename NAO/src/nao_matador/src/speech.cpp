/*
 *   @author: Jaime Andrés González Eelman                                                           
 *   @author: Jesús Andrés Varona                                                                    
 *   @author: Sherif Shousha                                                                         
 *   @author: Wenlan Shen                                                                            
 *                                                                                                   
 *   This section sets up the servers, publishers and subscribers necessary for the speech recgonition and generation                                                          
 */

#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdlib.h>
#include <ros/ros.h>
#include "message_filters/subscriber.h"
#include <string.h>
#include <naoqi_bridge_msgs/HeadTouch.h>
#include <naoqi_bridge_msgs/HandTouch.h>
#include <std_srvs/Empty.h>
#include <boost/algorithm/string.hpp>
#include <boost/date_time.hpp>
#include <naoqi_bridge_msgs/SpeechWithFeedbackActionGoal.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <naoqi_bridge_msgs/SetSpeechVocabularyActionGoal.h>
#include <naoqi_bridge_msgs/WordRecognized.h>
#include <std_msgs/Bool.h>
#include "actionlib/client/simple_action_client.h"
#include <nao_matador/SetSpeechVocabularyAction.h>
#include <nao_matador/SpeechWithFeedbackAction.h>
#include <nao_matador/Speaking_msgs.h>

using namespace std;

bool stop_thread=false;
void spinThread()
{
  while(!stop_thread)
  {

    ros::spinOnce();
  }
}

class Nao_speech
{

protected:

  // ROS node handler
  ros::NodeHandle nh_;

  // Publisher for nao speech
  ros::Publisher speech_pub;

  // Publisher for nao vocabulary parameters
  ros::Publisher voc_params_pub;

  // Client for starting speech recognition
  ros::ServiceClient recog_start_srv;

  // Client for stoping speech recognition
  ros::ServiceClient recog_stop_srv;

  // Subscriber to speech recognition
  ros::Subscriber recog_sub;

  // Server for speaking
  ros::ServiceServer speak_serv;

  // Publisher for heard command
  ros::Publisher heard_pub;

  std_srvs::Empty srv_empty;
  vector<string> recog_words;

  nao_matador::SpeechWithFeedbackGoal speechGoal;

public:
/**
 * variables to detect if the operator has spoken and has been heard
 */
  bool speaking = false;
  bool heard = false;
  boost::thread *spin_thread;

  // Create the action client
  actionlib::SimpleActionClient<nao_matador::SetSpeechVocabularyAction> my_vocClient;
  actionlib::SimpleActionClient<nao_matador::SpeechWithFeedbackAction> my_speechClient;

  // Init the action client: target the /blink action server. Here "true" causes the action client to spin its own thread
  Nao_speech(): my_vocClient(nh_, "/speech_vocabulary_action", true), my_speechClient(nh_, "/speech_action", true)
  {
    // Speaking service
    speak_serv = nh_.advertiseService("/speaking", &Nao_speech::speak, this);
    ROS_INFO("Speaking server started");
    heard_pub = nh_.advertise<std_msgs::Bool>("/heard_command",1);
    speech_pub = nh_.advertise<naoqi_bridge_msgs::SpeechWithFeedbackActionGoal>("/speech_action/goal", 1);
    voc_params_pub= nh_.advertise<naoqi_bridge_msgs::SetSpeechVocabularyActionGoal>("/speech_vocabulary_action/goal", 1);
    recog_start_srv=nh_.serviceClient<std_srvs::Empty>("/start_recognition");
    recog_stop_srv=nh_.serviceClient<std_srvs::Empty>("/stop_recognition");
    recog_sub=nh_.subscribe("/word_recognized",1, &Nao_speech::speechRecognitionCallback, this);
    recog_words.clear();

    stop_thread=false;
    spin_thread=new boost::thread(&spinThread);
  }
  ~Nao_speech()
  {
    ROS_WARN_STREAM("Destroy the object.");
    stop_thread=true;
    sleep(1);
    spin_thread->join();
  }
/**
 * Function to get the the words nao should recognize 
 */
  void setVoc()
  {
      // set the speech vocabulary
      nao_matador::SetSpeechVocabularyGoal vocGoal;
      vocGoal.words = {"adelante", "detente", "pizza"};
      my_vocClient.sendGoalAndWait(vocGoal);
      ROS_INFO_STREAM("Vocabularies are set.");
  }
/**
 * Callback function when nao has detected the bull, and recognized operator´s voice command
 * returns a boolean indicating if the command has been heard.
 */
  void speechRecognitionCallback(const naoqi_bridge_msgs::WordRecognized::ConstPtr& msg)
  {
    recog_words.push_back(msg->words[0]);
    ROS_INFO_STREAM("I heard: "<< recog_words[0]);
    std_msgs::Bool heard;
    heard.data = false;
    if (recog_words[0].compare("adelante")==0)
    {

        heard.data = true;
        // heard_pub.publish(heard);
        
        // End recognition
        ROS_WARN_STREAM("End recognition.");
        recog_stop_srv.call(srv_empty);
    }
    heard_pub.publish(heard);
    recog_words.clear();
  }
/**
 * Boolean returning that nao is speaking.
 */
  bool speak(nao_matador::Speaking_msgs::Request &req, nao_matador::Speaking_msgs::Response &res)
  {
      // Speak the sentence the client requested
      string sentence ="";
      for (auto it = req.sentence.cbegin(); it != req.sentence.cend(); ++it)
      {        
        ROS_INFO_STREAM("say "<< *it);
        sentence+= *it;
        speechGoal.say = *it;
        my_speechClient.sendGoalAndWait(speechGoal);
      }
      ROS_INFO_STREAM("Finished speech");
      // Start recognition
      if(sentence.compare("Preparado")==0){
          ROS_WARN_STREAM("Start recognition.");
          recog_start_srv.call(srv_empty);
      }

      return res.speaking = true;
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
  ros::init(argc, argv, "module_speech");

  // Enter rosspin loop
  Nao_speech TermiNAOtor;
  ROS_INFO_STREAM("Started the Client");
  TermiNAOtor.my_vocClient.waitForServer();
  ROS_INFO_STREAM("Vocabulary Server started");
  TermiNAOtor.setVoc();

  ros::spin();

  return 0;
}
