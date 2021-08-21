#!/usr/bin/env python

"""  
    @author: Jaime Andrés González Eelman
    @author: Jesús Andrés Varona 
    @author: Sherif Shousha
    @author: Wenlan Shen 

    This module serves two purposes. First to handle the motion requests from the central state machine.
    Secondly, to handle the frame transformations form the camera optical frames to the Torso frame, required
    by the aruco visual algorithms from the vision module.
"""

import rospy
import time
import almath
import sys
from naoqi import ALProxy
import motion
from nao_matador.srv import Motion_msgs
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
import numpy as np


motionProxy = None
bull_angle_pub = None
bull_distance_pub = None


def movement(chains, names, angles, times):
    """
        Wrapper function for the different types of inputs
    :param chains: String[], chains of joints to control
    :param names: String[], joints to move
    :param angles: float[], desired angles for the joint movements
    :param times: float[], desired times for the joint movements
    """
    angles = [(np.array(a) * almath.TO_RAD).tolist() for a in angles]
    motionProxy.setStiffnesses(chains, 1.0)
    motionProxy.angleInterpolation(names, angles, times, True)


def move_handle(req):
    """
        Callback function that handles all of the movement requests from the central state machine
    :param req: service message, includes the actions to be performed, and the search angles for search moves
    """
    action = req.action
    angles = req.angles
    for act in action:
        if act == 'start':
            rospy.logwarn('MOVING: Start')
            postureProxy.goToPosture("StandInit", 0.75)
            movement(['Head'], ['HeadPitch'], [20.0], 1)
        if act == 'move':
            rospy.logwarn('MOVING: Move')
            movement(['Head'], ['HeadPitch'], [20.0], 1)
            movement(['LArm', 'RArm'], ["LArm", 'RArm'],
                     [30, 70.0, 0.0, -44.0, 0.0, 1.0, 30.0, 17.0, 0.0, 80.0, 0.0, 1.0], 2.0)
            time.sleep(0.5)
            movement(['Head'], ['HeadYaw'], [45.0], 1)
            movement(['LArm', 'RArm'], ['LShoulderPitch', 'RShoulderPitch'],
                     [[0.0, 30.0, 0.0, 30.0], [0.0, 30.0, 0.0, 30.0]], [[1.0, 2.0, 3.0, 4.0], [1.0, 2.0, 3.0, 4.0]])
        elif act == 'juke':
            rospy.logwarn('MOVING: Juke')
            movement(['LArm', 'RArm'], ["LArm", 'RArm'],
                     [30.0, -17.0, 0.0, -80.0, 0.0, 1.0, 30, -70.0, 0.0, 44.0, 0.0, 1.0], 0.5)
        elif act == 'challenge':
            rospy.logwarn('MOVING: Challenge')
            movement(['LArm', 'RArm'], ["LArm", 'RArm'],
                     [0.0, 70.0, 0.0, -80.0, 0.0, 1.0, 0.0, -70.0, 0.0, 80.0, 0.0, 1.0], 2.0)
            time.sleep(1.5)
            movement(['LArm', 'RArm'], ['LShoulderPitch', 'RShoulderPitch'],
                     [[-45.0, 45.0, -45.0, 45.0], [-45.0, 45.0, -45.0, 45.0]],
                     [[0.5, 1.0, 1.5, 2.0], [0.5, 1.0, 1.5, 2.0]])
        elif act == 'hurra':
            rospy.logwarn('MOVING: Hurra')
            movement(['LLeg', 'RLeg'], ['LHipPitch', 'RHipPitch'], [0.0, 0.0], 2.5)
            movement(['LArm', 'RArm'], ["LArm", 'RArm'],
                     [0.0, 0.0, -90.0, -80.0, 0.0, 1.0, 0.0, 0.0, 90.0, 80.0, 0.0, 1.0], 1.5)
            time.sleep(1.5)
            movement(['LLeg', 'RLeg'], ['LHipPitch', 'RHipPitch'], [-20.0, -20.0], 1)
        elif act == 'sad':
            rospy.logwarn('MOVING: Sad')
            movement(['LArm', 'RArm'], ["LArm", 'RArm'], [60.0, 0.0, 0.0, 0.0, 0.0, 1.0, 60.0, 0.0, 0.0, 0.0, 0.0, 1.0],
                     1.0)
            movement(['Head'], ['HeadYaw', 'HeadPitch'],
                     [[30.0, -30.0, 30.0, -30.0, 30, 0], [25.0, 25.0, 25.0, 25.0, 25.0, 0.0]],
                     [[0.75, 1.5, 2.25, 3.0, 3.75, 4.5], [0.75, 1.5, 2.25, 3.0, 3.75, 4.5]])
        elif act == 'down':
            movement(['Head'], ['HeadPitch'], [30.0], 1)
        elif act == 'front':
            movement(['Head'], ['HeadYaw'], [0.0], 1)
        elif act == 'right':
            movement(['Head'], ['HeadYaw'], [-90.0], 1)
        elif act == 'search':
            rospy.logwarn('MOVING: Searching')
            movement(['Head'], ['HeadPitch'], [30.0], 0.5)
            movement(['Head'], ['HeadYaw'], [angles[0]], 0.05)
        elif act == 'search bull':
            rospy.logwarn('MOVING: Searching the bull')
            movement(['Head'], ['HeadPitch'], [30.0], 1.0)
            movement(['Head'], ['HeadYaw'], [angles[0]], 0.1)

    return True


def transform(vector_raw, camera_frame_raw):
    """
        Helper function that performs the bulk of the math for the torso frame transformation
    :param vector_raw: vector to be transformed (If size bigger than 3, average into one vector)
    :param camera_frame_raw: camera frame from wich to perform the transformation
    :return: transformed vector
    """

    # Transformation from the camera optical frame to the camera frame, calculated by hand
    T = (np.array([0, 0, 1, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 1])).reshape(4, 4)
    camera_frame = (np.asarray(camera_frame_raw)).reshape(4, 4)

    # Formatting
    vector = np.asarray(vector_raw.data)
    if vector.size == 6:
        vectors = np.array_split(vector, 2)
        vector = np.add(vectors[0], vectors[1]) / 2
    if vector.size == 9:
        vectors = np.array_split(vector, 3)
        vector = np.add(np.add(vectors[0], vectors[1]), vectors[2]) / 3
    aruco_vector = np.append(vector, 1)

    # Calculate the aruco frame in torso coordinates
    return np.dot(camera_frame, np.dot(T, aruco_vector))



def aruco_callback_top(input):
    """
        This callback function deals with the information from the vision module, it calculates the vector of the marker in
        coordinates relative to the torso, publishes the yaw angle to  the bull
    :param input: vector with coordinates relative to the top camera optical frame
    """

    # Transform the vector coordinates to torso coordinates
    aruco_cords = transform(input, motionProxy.getTransform("CameraTop", motion.FRAME_TORSO, True))
    rospy.loginfo("TOP CAMERA: Marker transformation")

    # Wrap up
    bull_angle = Float32()
    bull_angle.data = (-1) * (np.arctan(aruco_cords[1] / aruco_cords[2]) * 180 / np.pi - 57 ) * 0.8 # The 0.8 is to make the movement less drastic and the -57 is an offset to target the center of the camera (Found empirically)

    # Publishing the results
    bull_angle_pub.publish(bull_angle)
    rospy.loginfo("Angle:  %f", bull_angle.data)


# This callback function deals with the information from the vision module, it calculates the of the marker in
# coordinates relative to the torso
def aruco_callback_bot(input):
    """
        This callback function deals with the information from the vision module, it calculates the vector of the marker in
        coordinates relative to the torso, publishes the distance to the bull
    :param input: vector with coordinates relative to the bottom camera optical frame
    """

    # Transform the vector coordinates to torso coordinates
    aruco_cords = transform(input, motionProxy.getTransform("CameraBottom", motion.FRAME_TORSO, True))
    rospy.loginfo("BOTTOM CAMERA: Marker transformation")

    # Wrap up
    bull_distance = Float32()
    bull_distance.data = (-1)*aruco_cords[2]

    # Publishing the results
    bull_distance_pub.publish(bull_distance)
    rospy.loginfo("Distance:  %f", bull_distance.data)


if __name__ == '__main__':
    """
        Main function, sets up the node, the communication with the other nodes and defines the callback functions
    """

    robotIP = str(sys.argv[1])
    PORT = int(sys.argv[2])

    motionProxy = ALProxy("ALMotion", robotIP, PORT)
    postureProxy = ALProxy("ALRobotPosture", robotIP, PORT)

    # Setting up th motionProxy
    motionProxy.wakeUp()
    motionProxy.setFallManagerEnabled(True) # Flag for fall recovery
    motionProxy.setCollisionProtectionEnabled("Arms", True) # Flag for collision avoidance

    # Setting up the publishers & subscribers
    rospy.init_node('motion_server')
    rospy.Subscriber("/aruco_data_top", Float32MultiArray, aruco_callback_top)
    rospy.Subscriber("/aruco_data_bot", Float32MultiArray, aruco_callback_bot)
    bull_angle_pub = rospy.Publisher('/bull_angle', Float32, queue_size=1)
    bull_distance_pub = rospy.Publisher('/bull_distance', Float32, queue_size=1)
    center_detected_pub = rospy.Publisher('/center_detected', Bool, queue_size=1)
    bull_detected_pub = rospy.Publisher('/center_detected', Bool, queue_size=1)
    rospy.Service('/motion', Motion_msgs, move_handle)

    rospy.spin()
