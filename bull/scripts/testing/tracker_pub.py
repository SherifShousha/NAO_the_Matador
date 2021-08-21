#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16MultiArray
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np

(major_ver, minor_ver, subminor_ver) = (cv2.__version__).split('.')

def tracker(bbox_publisher):

    # Declare a message
    obj_pos = Int16MultiArray()

    # Set up tracker.
    tracker_types = ['BOOSTING', 'MIL','KCF', 'TLD', 'MEDIANFLOW', 'GOTURN', 'MOSSE', 'CSRT']
    tracker_type = tracker_types[2]

    if int(minor_ver) < 3:
        #tracker = cv2.Tracker_create(tracker_type)
        if tracker_type == tracker_types[0]:
            tracker = cv2.TrackerBoosting_create()
        elif tracker_type == tracker_types[1]:
            tracker = cv2.TrackerMIL_create()
        elif tracker_type == tracker_types[2]:
            tracker = cv2.TrackerKCF_create()
        elif tracker_type == tracker_types[3]:
            tracker = cv2.TrackerTLD_create()
        elif tracker_type == tracker_types[4]:
            tracker = cv2.TrackerMedianFlow_create()
        elif tracker_type == tracker_types[5]:
            tracker = cv2.TrackerGOTURN_create()
        elif tracker_type == tracker_types[6]:
            tracker = cv2.TrackerMOSSE_create()
        elif tracker_type == tracker_types[7]:
            tracker = cv2.TrackerCSRT_create()
    else:
        if tracker_type == 'BOOSTING':
            tracker = cv2.TrackerBoosting_create()
        if tracker_type == 'MIL':
            tracker = cv2.TrackerMIL_create()
        if tracker_type == 'KCF':
            tracker = cv2.TrackerKCF_create()
        if tracker_type == 'TLD':
            tracker = cv2.TrackerTLD_create()
        if tracker_type == 'MEDIANFLOW':
            tracker = cv2.TrackerMedianFlow_create()
        if tracker_type == 'GOTURN':
            tracker = cv2.TrackerGOTURN_create()
        if tracker_type == 'MOSSE':
            tracker = cv2.TrackerMOSSE_create()
        if tracker_type == "CSRT":
            tracker = cv2.TrackerCSRT_create()

    # initialize the camera and grab a reference to the raw camera capture
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 32
    rawCapture = PiRGBArray(camera, size = (640, 480))
    # allow the camera to warmup
    time.sleep(0.1)

    # Show frame and select ROI
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        
        image = frame.array
        cv2.imshow("Image", image)

        # Click "s" on keyboard to select ROI
        if cv2.waitKey(1) & 0xFF == ord('s'):
            cv2.destroyAllWindows()
            bbox = cv2.selectROI(image)
            cv2.destroyAllWindows()
            rawCapture.truncate(0)
            break
        rawCapture.truncate(0)

    # Initialize tracker with first frame and bounding box
    ok = tracker.init(image, bbox)

    # cv2.namedWindow('Tracking', flags=cv2.WINDOW_NORMAL)

    # Loop for tracking 
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        
        image = frame.array

        # Start timer
        timer = cv2.getTickCount()  

        # Update tracker
        ok, bbox = tracker.update(image)

        # Calculate Frames per second (FPS)
        fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)

        # Draw bounding box
        if ok:
            # Tracking success
            p1 = (int(bbox[0]), int(bbox[1]))
            p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
            cv2.rectangle(image, p1, p2, (255,0,0), 2, 1)
            # Find the mid-point of this rectangle
            mid = (int(bbox[0] + (bbox[2])/2), int(bbox[1] + (bbox[3])/2))
            cv2.rectangle(image, mid, mid, (255,0,0), 2, 1)
            
            # Publish the bounding box position
            obj_pos.data = [mid[0], mid[1]]
            bbox_publisher.publish(obj_pos)
            rate.sleep()
        else :
            # Tracking failure
            cv2.putText(image, "Tracking failure detected", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)

        # Display tracker type on frame
        cv2.putText(image, tracker_type + " Tracker", (100,20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50),2)

        # Display FPS on frame
        cv2.putText(image, "FPS : " + str(int(fps)), (100,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2)

        # Display result
        cv2.imshow("Tracking", image)

        rawCapture.truncate(0)
        # Exit if ESC pressed
        k = cv2.waitKey(1) & 0xff
        if k == 27 : break

    cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        rospy.init_node('camera_node', anonymous=True)
        bbox_publisher = rospy.Publisher('object_position', Int16MultiArray, queue_size=10)
        rate = rospy.Rate(1)
        tracker(bbox_publisher)
    except rospy.ROSInterruptException:
        pass
