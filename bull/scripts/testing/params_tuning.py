#!/usr/bin/python

### TODO:
## 1. Use camshift to track NAO, publish NAO position for bull head tracking
      # TODO:  
            # 1. Add trackbar to find best parameters
            # 2. Replace selectROI function with templates and hard-coded roi
## 2. Use erode/dilate and find contour to detect red cloth, publish a boolean value
## 3. Led detection
## 4. Service/Client or Publisher/Subscriber ?
      # The former might be better to select from the 3 methods(nao search, red cloth 
      # search and led detection), then TODO: write a server for camera node and replace
      # the subscriber with a client in the main function


### The following is a demo for camshift method + publisher

# import the necessary packages
import rospy
from std_msgs.msg import Float32MultiArray
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np


# fake function to use it in "createTrackbar" function
def on_trackbar():
    print("")  


def search_for_nao(nao_pos_pub):
    #box and cloth box have to be the same size of the camera we present here camera with 640 pixel width and 420 pixel height
    x, y, w, h = 158, 41, (380-158), (422-41)   # template.jpg
    # x, y, w, h = 172, 101, (352-172), (387-101)   # template2.jpg
    x, y, w, h = 185, 76, (456-185), (477-76)  #template3.jpg
    box = (x, y, w, h)

    #create trackbars
    cv2.namedWindow("Controls", 1)
    cv2.createTrackbar("L-H", "Controls", 0, 180, on_trackbar)
    cv2.createTrackbar("L-S", "Controls", 0, 256, on_trackbar)
    cv2.createTrackbar("L-V", "Controls", 0, 256, on_trackbar)
    cv2.createTrackbar("U-H", "Controls", 180, 180, on_trackbar)
    cv2.createTrackbar("U-S", "Controls", 255, 256, on_trackbar)
    cv2.createTrackbar("U-V", "Controls", 255, 256, on_trackbar)
    cv2.createTrackbar("Thresh", "Controls", 0, 255, on_trackbar)
    cv2.createTrackbar("valMax", "Controls", 255, 255, on_trackbar)
    cv2.createTrackbar("Type", "Controls",0, 5, on_trackbar)
    cv2.createTrackbar("cam_Thresh", "Controls", 0, 255, on_trackbar)
    cv2.createTrackbar("cam_valMax", "Controls", 255, 255, on_trackbar)
    cv2.createTrackbar("cam_Type", "Controls", 0, 5, on_trackbar)


    # initialize the camera and grab a reference to the raw camera capture
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 32
    rawCapture = PiRGBArray(camera, size = (640, 480))
    # allow the camera to warmup
    time.sleep(0.1)

    # # Show frame and select ROI
    # for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        
    #     image = frame.array
    #     cv2.imshow("Image", image)

    #     # Click "s" on keyboard to select ROI
    #     if cv2.waitKey(1) & 0xFF == ord('s'):
    #         cv2.destroyAllWindows()
    #         bbox = cv2.selectROI(image)
    #         cv2.destroyAllWindows()
    #         rawCapture.truncate(0)
    #         break

    #     rawCapture.truncate(0)
    # Read template picture
    temp = cv2.imread("/home/pi/catkin_ws/src/bull/scripts/template3.jpg")

    blurred_temp = cv2.GaussianBlur(temp, (11, 11), 0)
    roi = blurred_temp[y:y+h, x:x+w]
    cv2.imshow("roi", roi)
    hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    
    # # set up the ROI for tracking
    # roi = image[int(bbox[1]):int(bbox[1]+bbox[3]), int(bbox[0]):int(bbox[0]+bbox[2])]
    # hsv_roi =  cv2.cv2tColor(roi, cv2.COLOR_BGR2HSV)
    # mask = cv2.inRange(hsv_roi, np.array((0., 60.,32.)), np.array((180.,255.,255.)))
    # roi_hist = cv2.calcHist([hsv_roi],[0],mask,[180],[0,180])
    # cv2.normalize(roi_hist,roi_hist,0,255,cv2.NORM_MINMAX)

    # Setup the termination criteria, either 10 iteration or move by atleast 1 pt
    term_crit = ( cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1 )

    # Loop for tracking 
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        
        image = frame.array

        # get the trackbars value
        l_h = cv2.getTrackbarPos("L-H", "Controls")
        l_s = cv2.getTrackbarPos("L-S", "Controls")
        l_v = cv2.getTrackbarPos("L-V", "Controls")
        u_h = cv2.getTrackbarPos("U-H", "Controls")
        u_s = cv2.getTrackbarPos("U-S", "Controls")
        u_v = cv2.getTrackbarPos("U-V", "Controls")
        t_th = cv2.getTrackbarPos("Thresh", "Controls")
        t_v = cv2.getTrackbarPos("valMax", "Controls")
        t_T = cv2.getTrackbarPos("Type", "Controls")
        cam_th = cv2.getTrackbarPos("cam_Thresh", "Controls")
        cam_v = cv2.getTrackbarPos("cam_valMax", "Controls")
        cam_T = cv2.getTrackbarPos("cam_Type", "Controls")

        

        # blur the image and then convert cam and image (RGB/BGR) to HSV (hue saturation value) with H range 0..180
        
        blurred_cam = cv2.GaussianBlur(image, (11, 11), 0)
        
        cam_hsv = cv2.cvtColor(blurred_cam, cv2.COLOR_BGR2HSV)

        # calculate hue, saturation, value
        h, s, v = cv2.split(hsv_roi)

        # @NOTE next four rows are just in case we want to use inRange instatt of threshold
        lower_hsv = np.array([l_h, l_s, l_v])
        higher_hsv = np.array([u_h, u_s, u_v])
        # mask = cv2.inRange(hsv_roi, lower_hsv, higher_hsv)
        # mask = cv2.inRange(hsv_roi, np.array((5., 60.,10.)), np.array((15.,255.,255.)))
        mask = cv2.inRange(hsv_roi, np.array((0., 165.,165.)), np.array((10.,255.,255.)))
        # find the mask of the region of interest
        # ret1, mask = cv2.threshold(s, t_th, 255, 0)
        # filter the mask
        mask = cv2.erode(mask, None, iterations=2)
        
        mask = cv2.dilate(mask, None, iterations=2)
        # cv2.imshow("Img mask after filters", mask)

        # Setup the termination criteria, either 10 iteration or move by atleast 1 pt
        term_crit = ( cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1 )

        # calculate the histogram and camshift if the mask is not empty
        if mask.size != 0:
            # calculate the histogram from saturation channel
            roi_hist = cv2.calcHist([hsv_roi],[0], mask ,[180],[0,180])
            cv2.normalize(roi_hist,roi_hist,0,255,cv2.NORM_MINMAX)

            # calculate the back project
            dst = cv2.calcBackProject([cam_hsv],[0],roi_hist, [0,180],1)
            # cv2.imshow("calc backproject", dst)
            # filter the dst 
            _, mask2 = cv2.threshold(dst, 15, 255, 0 )
            # cv2.imshow("threshold", mask2)
            # mask2 = cv2.erode(mask2, None, iterations=2)
            # # cv2.imshow("erode 2", mask2)
            # mask2 = cv2.dilate(mask2, None, iterations=2)
            # cv2.imshow(" calc backproject after filter ", mask2)
            
            # @NOTE: we can use that too to cancle the noise from the mask but it is not needed right now maybe in the lab
            # kernel = np.ones((5,5), np.uint8)
            # morphOpen = cv2.morphologyEx(mask2 , cv2.MORPH_OPEN, kernel)
            # cv2.imshow("open", morphOpen)

            # apply the meanshift to get the location
            ret, track_window = cv2.CamShift(mask2, box, term_crit)
            # Draw it on image
            pts = cv2.boxPoints(ret)
            pts = np.int0(pts)

            # check if the robot is detected by test the camshift outcome
            if  pts[0].all() != 0:
                # Calculate object position
                x1, y1 = pts[0]
                x3, y3 = pts[2]
                print(x1,y1,x3,y3)
                bw = abs(x1 - x3)
                bh = abs(y1 - y3)
                area = bw * bh
                print(area)
                print("NAO detected")
                rot_rect = cv2.polylines(image,[pts],True, 255,2)

                # Calculate object position
                x1, y1 = pts[0]
                x2, y2 = pts[1]
                x3, y3 = pts[2]
                x4, y4 = pts[3]
                # print(x1, y1, x2, y2, x3, y3, x4, y4)
                object_center = ((x1+x2+x3+x4)/4, (y1+y2+y3+y4)/4)
                # print("Object Center x:{} y:{}".format(object_center[0], object_center[1]))
                # Calculate frame center
                frm_height, frm_width,_ = image.shape
                frame_center = (frm_width/2, frm_height/2)
                # print("Frame Center x:{} y:{}".format(frame_center[0], frame_center[1]))

                offset_x = object_center[0] - frame_center[0]
                offset_y = -(object_center[1] - frame_center[1]) 

                # Publish the offsets
                # Declare a message
                obj_pos = Float32MultiArray()  
                obj_pos.data = [offset_x, offset_y]
                nao_pos_pub.publish(obj_pos)
                rate.sleep()

                # #@NOTE: change this values from the trackbar
                red_mask = cv2.inRange(cam_hsv, lower_hsv, higher_hsv)
                # red_mask = cv2.inRange(cam_hsv, np.array([170, 100, 60]), np.array([180, 255, 255]))
                # cv2.imshow("red mask", red_mask)
                # filter the mask with blob detection
                kernel_red = np.ones((5,5), np.uint8)
                red_mask = cv2.dilate(red_mask, kernel_red, iterations=1)
                red_mask = cv2.erode(red_mask, kernel_red, iterations=2)
                red_mask = cv2.dilate(red_mask, kernel_red, iterations=1)
                cv2.imshow("red mask after filter", red_mask)
                contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                # find the largest contours if there is more than one
                if len(contours) != 0:
                    c = max(contours, key= cv2.contourArea)
                    ((x_red, y_red), radius) = cv2.minEnclosingCircle(c)
                    # ignore if the largest contours radius small than 10
                    if radius > 10:
                        print("red detected")
                        # x, y are the center of the detected object 
                        cv2.circle(image, (int(x_red), int(y_red)), int(radius), (0, 255, 255), 2)
                        cv2.circle(image, (int(x_red), int(y_red)), 1, (255, 255, 255), 1)
                        print("x : ", x_red)
                        print("y:  ", y_red)
            else:
                print("nao is not detected!")
                    
            cv2.imshow('Image', image)

        
        
        k = cv2.waitKey(1) & 0xff
        # clear the stream in preparation for the next frame
        
        rawCapture.truncate(0)
        if k == 27:
            break
            
    cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        rospy.init_node('camera_node', anonymous=True)
        # NAO position publisher
        nao_pos_pub = rospy.Publisher('/nao_position', Float32MultiArray, queue_size=1)
        rate = rospy.Rate(1)
        search_for_nao(nao_pos_pub) 


    except rospy.ROSInterruptException:
        pass


