#!/usr/bin/python

# import the necessary packages
import rospy
from bull.msg import Tracking
from std_msgs.msg import Bool
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
import math


def filterLed(cam, params, name):
    """ 
    Color space filtering
    """
    hue = params[0]
    red = params[1]
    green = params[2]
    blue = params[3]

    hsv = cv2.cvtColor(cam, cv2.COLOR_BGR2HSV)
    hsvchannels = np.array(cv2.split(hsv))
    rgbchannels = np.array(cv2.split(cam))

    _, hue_mask = cv2.threshold(hsvchannels[0], hue[0], 255, cv2.THRESH_BINARY)
    _, hue_inv = cv2.threshold(hsvchannels[0], hue[1], 255, cv2.THRESH_BINARY_INV)

    _, red_mask = cv2.threshold(rgbchannels[2], red[0], 255, cv2.THRESH_BINARY)
    _, red_inv = cv2.threshold(rgbchannels[2], red[1], 255, cv2.THRESH_BINARY_INV)

    _, green_mask = cv2.threshold(rgbchannels[1], green[0], 255, cv2.THRESH_BINARY)
    _, green_inv = cv2.threshold(rgbchannels[1], green[1], 255, cv2.THRESH_BINARY_INV)

    _, blue_mask = cv2.threshold(rgbchannels[0], blue[0], 255, cv2.THRESH_BINARY)
    _, blue_inv = cv2.threshold(rgbchannels[0], blue[1], 255, cv2.THRESH_BINARY_INV)

    res_hue = np.multiply(hue_mask, hue_inv)
    res_red = np.multiply(red_mask, red_inv)
    res_green = np.multiply(green_mask, green_inv)
    res_blue = np.multiply(blue_mask, blue_inv)

    final_mask = np.multiply(res_hue, np.multiply(res_red, np.multiply(res_green, res_blue)))

    image = cv2.bitwise_and(cam, cam, mask=final_mask)

    # cv2.namedWindow(name)
    # cv2.imshow(name, image)

    # print(np.sum(final_mask))
    return np.sum(final_mask) >= 10

def calcFrec(cam, led):
    """ 
    Calculate frequency of LED blinking
    """
    global color
    global prev_color
    global cur_time
    global start_time
    global elapsed
    cur_time = time.time()
    if filterLed(cam, led, "Blue"):
        color = 1
        if prev_color == color:
            elapsed = cur_time - start_time
        else:
            start_time = cur_time
            rospy.loginfo("Detected the blue led")
    else:
        color = 0
        if prev_color != color:
            elapsed = cur_time - start_time
            frec = 0.5 / elapsed
            print(frec)
            #rospy.loginfo("Frecueny: ", str(frec))
            prev_color = color
            return frec
    prev_color = color
    return -1

def Bull_led(cam):
    """ LED detection """
    global challenged 
    # Color range of the killer look LED blinking
    led = [[85, 115], [30, 60], [190, 255], [190, 255]]
    frec = calcFrec(cam, led)
    # If it's blinking in the right frequency
    if 0.5 <= frec <= 0.8:
        rospy.loginfo("The matador is challenging me!")
        challenged = True
    elif frec != -1:
        rospy.loginfo("The matador is a coward!")
        challenged = False


def calculate_offset(img_width, img_height, object_center):
    '''
    Calculate the offset of object in the frame
    Range： [-1, 1]
    '''
    obj_x, obj_y = object_center
    offset_x = float(obj_x / img_width - 0.5) * 2
    offset_y = float(obj_y / img_height - 0.5) * 2

    return (offset_x, offset_y)


def camera_loop(nao_track_pub, red_track_pub, nao_challenge_pub):
    global challenged

    # Param: initial bounding box position
    x, y, w, h = 185, 76, (456-185), (477-76)  #template3.jpg
    track_window = (x, y, w, h)
    # track_window = (0, 0, 640, 480)
    # Param: camshift range
    nao_low_hsv = np.array((0., 165.,165.))
    nao_high_hsv = np.array((10.,255.,255.))
    # Param: red tracking hsv
    low_hsv = np.array([170, 80, 80])
    high_hsv = np.array([180, 255, 255])

    # msg object to be published
    nao_tracking = Tracking()
    red_tracking = Tracking()

    # initialize the camera and grab a reference to the raw camera capture
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 32
    rawCapture = PiRGBArray(camera, size = (640, 480))
    # allow the camera to warmup
    time.sleep(0.1)


    ### Preparation for camshift tracking
    # Load template picture
    temp = cv2.imread("/home/pi/catkin_ws/src/bull/scripts/template3.jpg")    

    # blur the image and then convert cam and image (RGB/BGR) to HSV (hue saturation value) with H range 0..180
    blurred_temp = cv2.GaussianBlur(temp, (11, 11), 0) 
    roi = blurred_temp[y:y+h, x:x+w]
    # cv2.imshow("ROI", roi)
    hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    # calculate hue, saturation, value
    h, s, v = cv2.split(hsv_roi)
    # find the mask of the region of interest
    # ret1, mask = cv2.threshold(s, t_th, 255, 0)
    mask = cv2.inRange(hsv_roi, nao_low_hsv, nao_high_hsv)
    # filter the mask
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    # cv2.imshow("Img mask after filters", mask)
    # print(mask.size)

    # Setup the termination criteria, either 10 iteration or move by atleast 1 pt
    term_crit = ( cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1 )


    # Video frame loop 
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        if rospy.is_shutdown():
            break

        image = frame.array

        frm_height, frm_width,_ = image.shape

        ###### LED COMMUNICATION 
        challenge = Bool()
        Bull_led(image)
        challenge = challenged
        nao_challenge_pub.publish(challenge)
        challenged = False

        ######## TRACKING
        # blur the image and then convert cam and image (RGB/BGR) to HSV (hue saturation value) with H range 0..180
        blurred_cam = cv2.GaussianBlur(image, (11, 11), 0)
        cam_hsv = cv2.cvtColor(blurred_cam, cv2.COLOR_BGR2HSV)
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
            mask2 = cv2.erode(mask2, None, iterations=1)
            # cv2.imshow("erode 2", mask2)
            mask2 = cv2.dilate(mask2, None, iterations=2)
            # cv2.imshow(" calc backproject after filter ", mask2)
            
            # @NOTE: we can use that too to cancle the noise from the mask but it is not needed right now maybe in the lab
            # kernel = np.ones((5,5), np.uint8)
            # morphOpen = cv2.morphologyEx(mask2 , cv2.MORPH_OPEN, kernel)
            # cv2.imshow("open", morphOpen)

            
            ret, track_window = cv2.CamShift(mask2, track_window, term_crit)
            # track_window = (0, 0, 640, 480)
            # Draw it on image
            pts = cv2.boxPoints(ret)
            pts = np.int0(pts)

            # check if the robot is detected by test the camshift outcome
            if  pts[0].all() != 0:
                rot_rect = cv2.polylines(image,[pts],True, 255,2)
                x1, y1 = pts[0]
                x2, y2 = pts[1]
                x3, y3 = pts[2]
                x4, y4 = pts[3]
                # Side length of the rectangular of bounding box
                l1 = math.sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1))
                l2 = math.sqrt((x3-x2)*(x3-x2) + (y3-y2)*(y3-y2))
                area = int(l1*l2)
                # print(l1, l2)
                # print(area)
                if area < 2000 or l1<40 or l2<40:
                    print("Reset window.")
                    track_window = (0, 0, 640, 480)
                if area > 6000 and l1 > 70 and l2 > 70:
                    print("NAO detected")
                    nao_tracking.detected = True
                    
                    # Calculate object position
                    nao_center = ((x2+x4)/2, (y2+y4)/2)
                    # print("Object Center x:{} y:{}".format(nao_center[0], nao_center[1]))

                    (nao_offset_x, nao_offset_y) = calculate_offset(frm_width, frm_height, nao_center)
                    # print("NAO offsets:   X: {} Y：{}".format(nao_offset_x, nao_offset_y)) 
                    
                    nao_tracking.x = nao_offset_x
                    nao_tracking.y = nao_offset_y

                else:
                    # nao not detected
                    rospy.loginfo("nao is not detected!")
                    nao_tracking.detected = False
                    nao_tracking.x = 0.0
                    nao_tracking.y = 0.0
                    # nao_track_pub.publish(nao_tracking)

        else:
            # nao not detected
            rospy.loginfo("nao is not detected!")
            nao_tracking.detected = False
            nao_tracking.x = 0.0
            nao_tracking.y = 0.0
            # nao_track_pub.publish(nao_tracking)    

        ### Track red cloth
        red_mask = cv2.inRange(cam_hsv, low_hsv, high_hsv)
        # red_mask2 = cv2.inRange(cam_hsv, low_hsv2, high_hsv2)
        # red_mask = red_mask1 + red_mask2
        # cv2.imshow("red mask", red_mask)
        # filter the mask with blob detection
        kernel_red = np.ones((5,5), np.uint8)
        red_mask = cv2.dilate(red_mask, kernel_red, iterations=1)
        red_mask = cv2.erode(red_mask, kernel_red, iterations=2)
        red_mask = cv2.dilate(red_mask, kernel_red, iterations=1)
        # cv2.imshow("red mask after filter", red_mask)
        contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # find the largest contours if there is more than one
        if len(contours) != 0:
            c = max(contours, key= cv2.contourArea)
            ((x_red, y_red), radius) = cv2.minEnclosingCircle(c)
            # ignore if the largest contours radius small than 50
            if radius > 100:
                print("red detected")
                red_tracking.detected = True 
                # x, y are the center of the detected object 
                cv2.circle(image, (int(x_red), int(y_red)), int(radius), (0, 255, 255), 2)
                cv2.circle(image, (int(x_red), int(y_red)), 1, (255, 255, 255), 1)
                # print("x: ", x_red)
                # print("y: ", y_red)

                red_center = (x_red, y_red)
                (red_offset_x, red_offset_y) = calculate_offset(frm_width, frm_height, red_center)
                # print("RED offsets:   X: {} Y：{}".format(red_offset_x, red_offset_y))

                red_tracking.x = red_offset_x
                red_tracking.y = red_offset_y
    
            else:
                # red cloth not detected
                red_tracking.detected = False
                red_tracking.x = 0.0
                red_tracking.y = 0.0
        else:
            # red cloth not detected
            red_tracking.detected = False
            red_tracking.x = 0.0
            red_tracking.y = 0.0

        cv2.imshow('Image', image)
        # Publish tracking results
        nao_track_pub.publish(nao_tracking)
        red_track_pub.publish(red_tracking)
     
        
        k = cv2.waitKey(1) & 0xff
        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)
        if k == 27:
            break
            
    cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        global color
        color = 0
        global prev_color
        prev_color = color
        global cur_time
        cur_time = time.time()
        global start_time
        start_time = cur_time
        global elapsed
        elapsed = cur_time - start_time
        global challenged
        challenged = False

        rospy.init_node('camera_node', anonymous=True)
        # NAO position publisher
        nao_track_pub = rospy.Publisher("/nao_tracking", Tracking, queue_size=1)
        # Red cloth position publisher
        red_track_pub = rospy.Publisher("/red_tracking", Tracking, queue_size=1)
        # NAO challenge publisher
        nao_challenge_pub = rospy.Publisher('/nao_challenge', Bool, queue_size=1)
        rate = rospy.Rate(2)
        camera_loop(nao_track_pub, red_track_pub, nao_challenge_pub)

    except rospy.ROSInterruptException:
        pass


