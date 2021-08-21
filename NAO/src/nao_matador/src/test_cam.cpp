#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream> // for converting the command line parameter to integer

int main(int argc, char** argv){
    // Normal ROS stuff
    ros::init(argc, argv, "test_cam");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("/nao_robot/camera/top/camera/image_raw", 1);

    // Set the index to your video device (Find out with: $ ls -ltrh /dev/video*)
    cv::VideoCapture cap(0);
    // Check if video device can be opened with the given index
    if(!cap.isOpened()) return 1;
    cv::Mat frame;
    sensor_msgs::ImagePtr msg;

    ros::Rate loop_rate(1);
    while (nh.ok()) {
        cap >> frame;
        // Check if grabbed frame is actually full with some content
        if(!frame.empty()) {
            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            pub.publish(msg);
        }
        ros::spinOnce();
    }
}