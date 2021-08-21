/*
 *   @author: Jaime Andrés González Eelman                                                           
 *   @author: Jesús Andrés Varona                                                                    
 *   @author: Sherif Shousha                                                                         
 *   @author: Wenlan Shen                                                                            
 *                                                                                                   
 *   This module is responsible for the vision in NAO.                                                           
*/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <aruco/aruco.h>
#include <iostream>
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "nao_matador/Detection_msg.h"
#include <chrono>

using namespace cv;
using namespace std;

// data publisher
ros::Publisher detection_top;
ros::Publisher detection_bottom;
ros::Publisher facing_pub;
ros::Publisher challenged_pub;
ros::Publisher aruco_data_top_pub;
ros::Publisher aruco_data_bot_pub;

// States for the Bull robot
int state;
int color;
int prev_color;
chrono::time_point<std::chrono::steady_clock> cur_time;
chrono::time_point<std::chrono::steady_clock> start_time;
chrono::duration<double> elapsed;

aruco::CameraParameters cameraParams()
{
    // create a variable in the class
    aruco::CameraParameters TheCameraParameters;

    // load the parameter matrix in the constructor
    Mat dist(1, 5, CV_32FC1);
    dist.at<float>(0, 0) = -0.066494;
    dist.at<float>(0, 1) = 0.095481;
    dist.at<float>(0, 2) = -0.000279;
    dist.at<float>(0, 3) = 0.002292;
    dist.at<float>(0, 4) = 0.000000;

    Mat cameraP(3, 3, CV_32FC1);
    cameraP.at<float>(0, 0) = 551.543059;
    cameraP.at<float>(0, 1) = 0.000000;
    cameraP.at<float>(0, 2) = 327.382898;
    cameraP.at<float>(1, 0) = 0.000000;
    cameraP.at<float>(1, 1) = 553.736023;
    cameraP.at<float>(1, 2) = 225.026380;
    cameraP.at<float>(2, 0) = 0.000000;
    cameraP.at<float>(2, 1) = 0.000000;
    cameraP.at<float>(2, 2) = 1.000000;

    TheCameraParameters.setParams(cameraP, dist, Size(640, 480));
    TheCameraParameters.resize(Size(640, 480));

    return TheCameraParameters;
}
/**
 * This Function is to filter the color space to detect LED.
 */
bool filter_led(Mat cam, int params[][2], string name)
{
    int *hue = params[0];
    int *red = params[1];
    int *green = params[2];
    int *blue = params[3];

    // Hsv mask
    Mat hsv;
    Mat hsvchannels[3];
    cvtColor(cam, hsv, CV_BGR2HSV);
    split(hsv, hsvchannels);

    Mat hue_mask, hue_mask_inv;

    // RGB mask
    Mat rgbchannels[3];
    split(cam, rgbchannels);

    Mat red_pic, green_pic, blue_pic;
    Mat red_inv, green_inv, blue_inv;

    threshold(hsvchannels[0], hue_mask, hue[0], 255, THRESH_BINARY);         // The third value must be calibrated to the image
    threshold(hsvchannels[0], hue_mask_inv, hue[1], 255, THRESH_BINARY_INV); // The third value must be calibrated to the image

    threshold(rgbchannels[2], red_pic, red[0], 255, THRESH_BINARY);     // The third value must be calibrated to the image
    threshold(rgbchannels[1], green_pic, green[0], 255, THRESH_BINARY); // The third value must be calibrated to the image
    threshold(rgbchannels[0], blue_pic, blue[0], 255, THRESH_BINARY);   // The third value must be calibrated to the image

    threshold(rgbchannels[2], red_inv, red[1], 255, THRESH_BINARY_INV);     // The third value must be calibrated to the image
    threshold(rgbchannels[1], green_inv, green[1], 255, THRESH_BINARY_INV); // The third value must be calibrated to the image
    threshold(rgbchannels[0], blue_inv, blue[1], 255, THRESH_BINARY_INV);   // The third value must be calibrated to the image

    Mat res_hue = hue_mask.mul(hue_mask_inv);

    Mat res_red = red_pic.mul(red_inv);
    Mat res_green = green_pic.mul(green_inv);
    Mat res_blue = blue_pic.mul(blue_inv);

    Mat final_mask = res_red.mul(res_green.mul(res_blue.mul(res_hue)));

    Mat image;
    bitwise_and(cam, cam, image, final_mask);

    namedWindow(name);
    imshow(name, image);

    return sum(final_mask)[0] >= 200;
}

/**
 * Function to detect LED´s color
 */

void Nao_led(Mat cam)
{
 /**
 * we used this trackbars to find the best parameters.
 */
 // get the trackbars value

//    int l_r = getTrackbarPos("L_R", "Controls");
//    int u_r = getTrackbarPos("U_R", "Controls");
//    int l_g = getTrackbarPos("L_G", "Controls");
//    int u_g = getTrackbarPos("U_G", "Controls");
//    int l_b = getTrackbarPos("L_B", "Controls");
//    int u_b = getTrackbarPos("U_B", "Controls");
//    int l_h = getTrackbarPos("lower", "Controls");
//    int u_h = getTrackbarPos("higher", "Controls");


    int red_led[4][2] = {{45 / 2, 360 / 2}, {227, 242}, {13, 26}, {23, 39}};
    int blue_led[4][2] = {{170 / 2, 200 / 2}, {58, 68}, {254, 256}, {245, 256}};

    bool blue = filter_led(cam, blue_led, "Blue");
    bool red = filter_led(cam, red_led, "Red");

    if (blue)
    {
        ROS_WARN_STREAM("detected the blue led!");
        color = 1;
    }
    if (red)
    {
        ROS_WARN_STREAM("detected the red led!");
        color = 2;
    }
    if (!blue && !red)
    {
        ROS_INFO("detected nothing :(");
        color = 0;
    }
}

/**
 *Function to determine if the bull is facing o challenging depending on the LED´s color
 */

void bullState(Mat cam)
{
    Nao_led(cam);

    std_msgs::Bool facing;
    std_msgs::Bool challenged;
    facing.data = false;
    challenged.data = false;

    if (color == 1)
        facing.data = true;
    else if (color == 2)
        challenged.data = true;

    facing_pub.publish(facing);
    challenged_pub.publish(challenged);
}

// Detect all markers in the target image of the top camera
Mat detectMarkerTop(Mat inputImage){
    Mat imageCopy;
    inputImage.copyTo(imageCopy);

    aruco::MarkerDetector Detector_bull;
    aruco::MarkerDetector Detector_center;
    vector<aruco::Marker> markers_bull;
    vector<aruco::Marker> markers_center;

    Detector_bull.detect(inputImage, markers_bull, cameraParams(), 0.045);
    Detector_center.detect(inputImage, markers_center, cameraParams(), 0.12);

    std_msgs::Float32MultiArray aruco_data;
    nao_matador::Detection_msg detection;
    detection.bull_detection = false;
    detection.aruco_detection = false;

    // Format the markers to be published
    // if the bull markers is detected
    if (markers_bull.size() > 0){
        detection.aruco_detection = true;
    
        for (size_t i = 0; i < markers_bull.size(); i++){
            ROS_INFO_STREAM("TOP CAMERA BIG: 3D position id:  " << markers_bull[i].id << ":\n"
                                                            << markers_bull[i].Tvec);

            if (markers_bull[i].id == 20 || markers_bull[i].id == 22 || markers_bull[i].id == 24){
                
                // set the detected marker as the bull markers
                detection.bull_detection = true;
                
                // get the aruco center and 3D translation
                aruco_data.data.push_back(markers_bull[i].getCenter().x);
                aruco_data.data.push_back(markers_bull[i].getCenter().y);
                aruco_data.data.push_back(markers_bull[i].Tvec.at<float>(0, 3));

                aruco_data_top_pub.publish(aruco_data);
            }
            else if (markers_bull[i].id == 50)
                detection.bull_detection = false;

            //draw  markers in the image
            markers_bull[i].draw(imageCopy, Scalar(0, 0, 255), 2);
        }
    }

    // if the arena cetner markers is detected
    if (markers_center.size() > 0){
        detection.aruco_detection = true;

        for (size_t i = 0; i < markers_center.size(); i++) {
            ROS_INFO_STREAM("TOP CAMERA SMALL: 3D position id:  " << markers_center[i].id << ":\n"
                                                                  << markers_center[i].Tvec);
            
            if (markers_center[i].id == 50) {
                // set the detected aruco as the arena center
                detection.bull_detection = false;

                // get the aruco center and 3D translation
                aruco_data.data.push_back(markers_bull[i].getCenter().x);
                aruco_data.data.push_back(markers_bull[i].getCenter().y);
                aruco_data.data.push_back(markers_bull[i].Tvec.at<float>(0, 3));

                //for (int j = 0; j < 3; ++j)
                    //aruco_data.data.push_back(markers_center[i].Tvec.at<float>(0, j));
                aruco_data_top_pub.publish(aruco_data);
            }
            else if (markers_center[i].id == 20 || markers_center[i].id == 22 || markers_center[i].id == 24){
                detection.bull_detection = true;
            }

            //draw  markers in the image
            markers_center[i].draw(imageCopy, Scalar(0, 0, 255), 2);
        }
    }

    detection_top.publish(detection);
    return imageCopy;
}

// Detect all markers in the target image of the bottom camera
Mat detectMarkerBot(Mat inputImage){
    Mat imageCopy;
    inputImage.copyTo(imageCopy);

    aruco::MarkerDetector Detector_bull;
    aruco::MarkerDetector Detector_center;
    vector<aruco::Marker> markers_bull;
    vector<aruco::Marker> markers_center;

    Detector_bull.detect(inputImage, markers_bull, cameraParams(), 0.045);
    Detector_center.detect(inputImage, markers_center, cameraParams(), 0.12);

    std_msgs::Float32MultiArray aruco_data;
    nao_matador::Detection_msg detection;
    detection.bull_detection = false;
    detection.aruco_detection = false;

    // Format the markers to be published
    // if the bull markers is detected
    if (markers_bull.size() > 0){
        detection.aruco_detection = true;

        for (size_t i = 0; i < markers_bull.size(); i++){
            ROS_INFO_STREAM("BOTTOM CAMERA: 3D position id:  " << markers_bull[i].id << ":\n"
                                                            << markers_bull[i].Tvec);

            if (markers_bull[i].id == 20 || markers_bull[i].id == 22 || markers_bull[i].id == 24) {
                // set the detected marker as the bull markers.
                detection.bull_detection = true;

                // set the 3D translations to calculate the distance
                for (int j = 0; j < 3; ++j)
                    aruco_data.data.push_back(markers_bull[i].Tvec.at<float>(0, j));
            }
            else if (markers_bull[i].id == 50)
                detection.bull_detection = false;

            //draw  markers in the image
            markers_bull[i].draw(imageCopy, Scalar(0, 0, 255), 2);
        }
        aruco_data_bot_pub.publish(aruco_data);
    }
    
    // if the arena cetner markers is detected
    if (markers_center.size() > 0){
        detection.aruco_detection = true;

        for (size_t i = 0; i < markers_center.size(); i++) {
            ROS_INFO_STREAM("BOTTOM CAMERA: 3D position id:  " << markers_center[i].id << ":\n"
                                                            << markers_center[i].Tvec);

            if (markers_center[i].id == 50) {
                // set the detected marker as the arena center
                detection.bull_detection = false;
                // set the 3D translations to calculate the distance
                for (int j = 0; j < 3; ++j)
                    aruco_data.data.push_back(markers_center[i].Tvec.at<float>(0, j));
            }
            else if (markers_center[i].id == 20 || markers_center[i].id == 22 || markers_center[i].id == 24)
                detection.bull_detection = true;

            //draw  markers in the image
            markers_center[i].draw(imageCopy, Scalar(0, 0, 255), 2);
        }
        aruco_data_bot_pub.publish(aruco_data);
    }

    detection_bottom.publish(detection);
    return imageCopy;
}

// Subscriber whenever there is a new image from the top camera, it creates a picture, then detects the leds,
// then detects markers and shows the results
// fake function to use it in "createTrackbar" function
void on_trackbar(int, void *)
{
}

/**
 * Callback function that subscribes to NAO´s top camera.
 */

void topCameraCallback(const sensor_msgs::ImageConstPtr &msg)
{
 /**
 * we used this trackbars to find the best parameters.
 */

//    //set the paramters for dynamic
//    int slider = 0;
//    int slider_max = 179;
//    int slider_2 = 179;
//    int slider_max_2 = 179;
//
//    int slider_3 = 0;
//    int slider_max_3 = 255;
//
//    int slider_4 = 179;
//    int slider_max_4 = 255;
//    int slider_5 = 179;
//    int slider_max_5 = 255;
//    int slider_6 = 179;
//    int slider_max_6 = 255;
//
//    int t_thresh = 0;
//    int t_thresh_max = 255;
//    int t_val = 0;
//    int t_val_max = 255;
//
//    int lower_heu = 0;
//    int lower_heu_max = 179;
//    int upper_heu = 179;
//    int upper_heu_max = 179;
//
//
//    namedWindow("Controls", 1);
//    createTrackbar("L_R", "Controls", &slider_3, slider_max_3, on_trackbar);
//    createTrackbar("U_R", "Controls", &slider_4, slider_max_4, on_trackbar);
//    createTrackbar("L_G", "Controls", &slider_5, slider_max_5, on_trackbar);
//    createTrackbar("U_G", "Controls", &slider_6, slider_max_6, on_trackbar);
//    createTrackbar("L_B", "Controls", &t_thresh, t_thresh_max, on_trackbar);
//    createTrackbar("U_B", "Controls", &t_val, t_val_max, on_trackbar);
//    createTrackbar("lower", "Controls", &lower_heu, lower_heu_max, on_trackbar);
//    createTrackbar("higher", "Controls", &upper_heu, upper_heu_max, on_trackbar);

    try
    {
        // Convert msg to opencv format
        Mat top_cam = cv_bridge::toCvShare(msg, "bgr8")->image;

        bullState(top_cam);

        Mat marker_image;
        marker_image = detectMarkerTop(top_cam);
        namedWindow("3D marker position top");
        imshow("3D marker position top", marker_image);
        char k = waitKey(30);
        if (k == 's')
            ros::shutdown();
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

/**
 * Callback function thats subscribes to NAO´s bottom camera
 */


void botCameraCallback(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        // Convert msg to opencv format
        Mat bot_cam = cv_bridge::toCvShare(msg, "bgr8")->image;

        Mat marker_image = detectMarkerBot(bot_cam);
        namedWindow("3D marker position bottom");
        imshow("3D marker position bottom", marker_image);
        char k = waitKey(30);
        if (k == 's')
            ros::shutdown();
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "module_vision");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    //// Setting up the subscribers and publishers
    // Subs
    image_transport::Subscriber top_sub = it.subscribe("/nao_robot/camera/top/camera/image_raw", 1, topCameraCallback);
    image_transport::Subscriber bot_sub = it.subscribe("/nao_robot/camera/bottom/camera/image_raw", 1, botCameraCallback);
    
    // custom message includs two booleans
    detection_top = nh.advertise<nao_matador::Detection_msg>("/detection_top", 1);
    detection_bottom = nh.advertise<nao_matador::Detection_msg>("/detection_bottom", 1);
    // Boolean
    facing_pub = nh.advertise<std_msgs::Bool>("/facing", 1);
    challenged_pub = nh.advertise<std_msgs::Bool>("/challenged", 1);
    // Floats
    aruco_data_top_pub = nh.advertise<std_msgs::Float32MultiArray>("/aruco_data_top", 1);
    aruco_data_bot_pub = nh.advertise<std_msgs::Float32MultiArray>("/aruco_data_bot", 1);

    ros::spin();
    destroyAllWindows();
}
