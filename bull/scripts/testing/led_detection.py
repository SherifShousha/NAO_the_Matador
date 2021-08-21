#!/usr/bin/python

""" This node is to for tuning the LED parameters on NAO"""

# import the necessary packages
import rospy
from std_msgs.msg import Bool
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np

# global color
# global prev_color
# global cur_time
# global start_time
# global elapsed
challenged = False

# fake function to use it in "createTrackbar" function
def on_trackbar(x):
    pass


def filterLed(cam, params, name):

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

    cv2.namedWindow(name)
    cv2.imshow(name, image)

    # print(np.sum(final_mask))
    return np.sum(final_mask) >= 10


def calcFrec(cam, led):
    global color
    global prev_color
    global cur_time
    global start_time
    global elapsed
    cur_time = time.time()

    if filterLed(cam, led, "Red"):
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
            print("Frecueny: ", frec)
            prev_color = color
            return frec
    prev_color = color
    return -1


def Bull_led(cam):
    global challenged
    l_h = cv2.getTrackbarPos("L-H", "Controls")
    l_r = cv2.getTrackbarPos("L-R", "Controls")
    l_g = cv2.getTrackbarPos("L-G", "Controls")
    l_b = cv2.getTrackbarPos("L-B", "Controls")
    u_h = cv2.getTrackbarPos("U-H", "Controls")
    u_r = cv2.getTrackbarPos("U-R", "Controls")
    u_g = cv2.getTrackbarPos("U-G", "Controls")
    u_b = cv2.getTrackbarPos("U-B", "Controls")

    # red_led = [[319.0/2, 359.0/2],[183.0, 223.0], [10.0, 30.0], [64.0, 104.0]]
    # led = [[l_h, u_h], [l_r, u_r], [l_g, u_g], [l_b, u_b]]
    # led = [[90, 110], [20, 70], [60, 255], [128, 255]]
    # led = [[85, 95], [20, 70], [170, 255], [170, 255]]
    led = [[85, 115], [30, 60], [190, 255], [190, 255]]
    
    frec = calcFrec(cam, led)
    if 0.5 <= frec <= 0.8:
        rospy.loginfo("The matador is challenging me!")
        challenged = True
        # return True
    elif frec != -1:
        rospy.loginfo("The matador is a coward!")
        challenged = False
        # return False


def led_detection(nao_challenge_pub):
    global challenged
    #create trackbars
    cv2.namedWindow("Controls", 1)
    cv2.createTrackbar("L-H", "Controls", 0, 179, on_trackbar)
    cv2.createTrackbar("L-R", "Controls", 0, 255, on_trackbar)
    cv2.createTrackbar("L-G", "Controls", 0, 255, on_trackbar)
    cv2.createTrackbar("L-B", "Controls", 0, 255, on_trackbar)
    cv2.createTrackbar("U-H", "Controls", 179, 179, on_trackbar)
    cv2.createTrackbar("U-R", "Controls", 255, 255, on_trackbar)
    cv2.createTrackbar("U-G", "Controls", 255, 255, on_trackbar)
    cv2.createTrackbar("U-B", "Controls", 255, 255, on_trackbar)


    # initialize the camera and grab a reference to the raw camera capture
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 32
    rawCapture = PiRGBArray(camera, size=(640, 480))
    # allow the camera to warmup
    time.sleep(0.1)


    # Video loop 
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        if rospy.is_shutdown():
            break

        image = frame.array
        cv2.imshow("Image", image)

        # LED COMMUNICATION
        challenge = Bool()
        Bull_led(image)
        challenge = challenged
        nao_challenge_pub.publish(challenge)

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

        rospy.init_node('camera_node', anonymous=True)
        # NAO position publisher
        nao_challenge_pub = rospy.Publisher('/nao_challenge', Bool, queue_size=1)

        led_detection(nao_challenge_pub)

    except rospy.ROSInterruptException:
        pass
