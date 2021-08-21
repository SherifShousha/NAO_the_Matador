#!/usr/bin/env python

"""
    @author: Jaime Andrés González Eelman
    @author: Jesús Andrés Varona 
    @author: Sherif Shousha
    @author: Wenlan Shen 

    Centralized integrator of the system. It is implemented in the structure of a state machine that cycles though the
    required states to complete the whole bull fighting routine
"""

import time
import rospy
from nao_matador.srv import Walking_msgs
from nao_matador.srv import Led_msgs
from nao_matador.srv import Motion_msgs
from nao_matador.msg import Detection_msg
from nao_matador.srv import Speaking_msgs
from nao_matador.srv import Audio_msgs
import std_msgs.msg as msgs
import numpy as np


class Module:
    """
        Parent class representing a module, all modules extend from it
    """

    def __init__(self):
        self.on = True


class VisionModule(Module):
    """
        Module object representing the system's vision, it handles communication to and from the node
    """

    def __init__(self):
        Module.__init__(self)
        self.aruco_detected_top = False
        self.bull_detected_top = False
        self.aruco_detected_bottom = False
        self.bull_detected_bottom = False
        self.challenged = False
        self.facing = False
        # self.bull_detected = False
        self.bull_angle = 0
        self.bull_distance = -1

        rospy.Subscriber("/detection_top", Detection_msg, self.detection_top_callback)
        rospy.Subscriber("/detection_bottom", Detection_msg, self.detection_bottom_callback)
        rospy.Subscriber("/bull_angle", msgs.Float32, self.angle_callback)
        rospy.Subscriber("/bull_distance", msgs.Float32, self.distance_callback)
        rospy.Subscriber("/facing", msgs.Bool, self.facing_callback)
        rospy.Subscriber("/challenged", msgs.Bool, self.challenge_callback)

    def detection_top_callback(self, input):
        """
            Callback function from the aruco detection algorithms of the top camera
        :param input: Booleans confirming detection
        """
        self.aruco_detected_top = input.aruco_detection
        self.bull_detected_top = input.bull_detection
        if self.aruco_detected_top:
            rospy.logwarn('The aruco has been detected TOP!')
            if self.bull_detected_top:
                rospy.logwarn(" Bull detected TOP!")

    def detection_bottom_callback(self, input):
        """
            Callback function from the aruco detection algorithms of the top camera
        :param input: Booleans confirming detection
        """
        self.aruco_detected_bottom = input.aruco_detection
        self.bull_detected_bottom = input.bull_detection
        if self.detected_bottom:
            rospy.logwarn('The aruco has been detected BOTTOM!')
            if self.bull_detected_bottom:
                rospy.logwarn("The Bull is detected BOTTOM")

    def facing_callback(self, input):
        """
            Callback function from the blue led detection
        :param input: Boolean confirming detection
        """
        self.facing = input.data
        if self.facing:
            rospy.logwarn('The bull has also seen the NAO')

    def challenge_callback(self, input):
        """
            Callback function from the red led detection
        :param input: Boolean confirming detection
        """
        self.challenged = input.data
        if self.challenged:
            rospy.logwarn('The bull is ready to attack!')

    def angle_callback(self, input):
        """
            Callback function from the motion_poxy with the angle calculated from the top camera
        :param input: Float with the angle for rotation
        """
        self.bull_angle = input.data
        rospy.loginfo('The aruco is %f to the right', input.data)

    def distance_callback(self, input):
        """
            Callback function from the motion_poxy with the distance calculated from the bottom camera
        :param input: Float with the distance from the robot
        """
        self.bull_distance = input.data
        rospy.loginfo('The aruco bull is %f m away', input.data)


class WalkingModule(Module):
    """
        Module object representing the system's walking, it handles communication to and from the node
    """

    def __init__(self):
        Module.__init__(self)
        self.walking_serv = rospy.ServiceProxy('/walking', Walking_msgs)

    def rotate(self, angle):
        """
            Sends a turning command through the /walking service
        :param angle: Float, angle of rotation in radians
        :return: Boolean, confirmation of action
        """
        return self.walking_serv(['turn'], [angle], 1)

    def sideways(self, distance):
        """
            Sends a walking sideways command through the /walking service
        :param distance: Float, distance in meters
        :return: Boolean, confirmation of action
        """
        return self.walking_serv(['side'], [distance], 1)

    def forward(self, distance):
        """
            Sends a walking forward (or backwards) command through the /walking service
        :param distance: Float, distance in meters
        :return: Boolean, confirmation of action
        """
        return self.walking_serv(['forward'], [distance], 1)


class LedModule(Module):
    """
        Module object representing the system's walking, it handles communication to and from the node
    """

    def __init__(self):
        Module.__init__(self)
        self.action = ""
        self.leds_serv = rospy.ServiceProxy('/leds', Led_msgs)

    def blink(self, input):
        """
            Sends a blinking command through the /leds service
        :param input: String, blinking routine to be executed
        :return: Boolean, confirmation of action
        """
        return self.leds_serv([input], 1)


class BumperModule(Module):
    """
        Module object representing the system's bumpers, it handles communication to and from the node
    """

    def __init__(self):
        Module.__init__(self)
        self.hit = False
        rospy.Subscriber("/hit", msgs.Bool, self.hit_callback)

    def hit_callback(self, input):
        """
            Callback function that updates the value of the hit variable
        :param input: Boolean, determining if there was a collision
        """
        self.hit = input.data
        if self.hit:
            rospy.logwarn('The NAO has been hit')


class MotionModule(Module):
    """
        Module object representing the system's motion, it handles communication to and from the node
    """

    def __init__(self):
        Module.__init__(self)
        self.mani_serv = rospy.ServiceProxy('/motion', Motion_msgs)

    def action(self, action, angles=0):
        """
            Sends a motion command through the /motion service
        :param input: String, motion routine to be executed
        :return: Boolean, confirmation of action
        """
        return self.mani_serv([action], 1, [angles])


class SpeechModule(Module):
    """
        Module object representing the system's voice recognition and generation, it handles communication to and from the node
    """

    def __init__(self):
        self.facing = False
        self.heard = False
        self.speaking_client = rospy.ServiceProxy('/speaking', Speaking_msgs)
        self.playing_client = rospy.ServiceProxy('/sound_generator', Audio_msgs)

        rospy.Subscriber("/heard_command", msgs.Bool, self.heard_callback)

    def speak(self, sentence):
        """
            Sends a sentence to be reproduced by the NAO's speech generation
        :param sentence: String, words to be spoken out by the NAO
        """
        self.speaking_client([sentence])

    def points(self, nao_points, bull_points):
        """
            This function takes in the current points in the system, so that the NAO speaks them outloud with
            correct spanish grammar.
        :param nao_points: Int, points/rounds that the NAO has won
        :param bull_points: Int, points/rounds that the Bull has won
        """
        if nao_points == 1:
            if bull_points == 1:
                sentence = "He ganado " + str(nao_points) + " ronda y el toro " + str(bull_points) + " ronda"
            else:
                sentence = "He ganado " + str(nao_points) + " ronda y el toro " + str(bull_points) + " rondas"
        else:
            if bull_points == 1:
                sentence = "He ganado " + str(nao_points) + " rondas y el toro " + str(bull_points) + " ronda"
            else:
                sentence = "He ganado " + str(nao_points) + " rondas y el toro " + str(bull_points) + " rondas"

        self.speak(sentence)

    def heard_callback(self, input):
        """
            Callback function that updates the value of the heard variable
        :param input: Boolean, determining if the "Adelante" command was heard
        """
        self.heard = input.data
        if self.heard:
            rospy.logwarn("Command received")

    def play_ole(self):
        """
            Sends the command to play "Ole!" to the audio_player_service
        """
        self.playing_client(['ole'])

    def play_caramba(self):
        """
            Sends the command to play "Ay caramba!" to the audio_player_service
        """
        self.playing_client(['caramba'])


def search_for_bull():
    """
        Helper function to search for the aruco markers of the bull by rotating the head at the beginning of each round.
        The head moves in a range of 90° and then moves 90° if the bull was not found.
    :return:
    """
    yaw_angle_bull = 0
    direction_bull = 1

    # Turn its head to search for the bull
    while not vision.bull_detected_top:
        motion.action('search bull', yaw_angle_bull)
        if direction_bull == 1:
            yaw_angle_bull = yaw_angle_bull + 10.0
            if yaw_angle_bull > 45.0:
                direction_bull = -1
        elif direction_bull == -1:
            yaw_angle_bull = yaw_angle_bull - 10.0
            if yaw_angle_bull < -45.0:
                direction_bull = 1
                walking.rotate((1.0 /2.0) * np.pi)
        if bumpers.hit:
            break

    rospy.logwarn('Detected the Bull!')


def search_for_aruco():
    """
        Helper function to detect the center of the arena by rotating the head and searching for the dedicated aruco marker.
    :return: Float, the angle in relation to the torso frame to the aruco marker.
    """
    pitch_angle = -20.0
    yaw_angle = 0.0
    direction = 1

    while (not vision.aruco_detected_top or
           (vision.aruco_detected_top and vision.bull_detected_top)):
        motion.action('search', yaw_angle)
        if direction == 1:
            yaw_angle = yaw_angle + 12
            if yaw_angle > 60.0:
                direction = -1
        elif direction == -1:
            yaw_angle = yaw_angle - 12
            if yaw_angle < -60.0:
                direction = 1
        if vision.aruco_detected_bottom and not vision.bull_detected_bottom:
            break

    return yaw_angle


def prep_machine(state):
    """
        Helper function to initialize the NAO. In state 1 the entrance routine is called and the robot goes to the center of the arena.
    :param state: Int, current state of the machine
    :return: Int, last state of the machine
    """

    # if everything is correct thus far run the entrance routine
    state = 1
    state_pub.publish(state)
    entrance_routine()

    # afterwards change to the looking for bull state
    state = 2
    state_pub.publish(state)
    return state


def entrance_routine():
    """
        Helper function to help the NAO walk to the center of the arena. He searches for the aruco marker in the center, rotates toward it
        then walks 20cm. Since the walking of the NAO is not straight, the routine is repeate until he detects the aruco marker with his bottom camera at
        a very short distance from him.
    """
    angle = 0
    while not vision.aruco_detected_bottom or \
            (vision.aruco_detected_bottom and vision.bull_detected_bottom):
        angle = search_for_aruco()
        walking.rotate(angle * np.pi / 180)
        time.sleep(3)
        walking.forward(0.2)


def run_machine(state):
    """
        Most important function of the project. Here the states are cycled while the NAO does the relevant actions to
        succeed in the encounter with bull.
    :param state: current state of the machine
    """

    # Initialize the points/rounds
    nao_points = 0
    bull_points = 0

    # While not in shutdown
    while cur_state != -1:
        try:
            rospy.loginfo('Current State: %i', state)

            # Start every loop returning to an inquisitive gaze and comming to the start postue
            if state == 2:
                motion.action('start')
                leds.blink('start')
                # go_to_center()
                search_for_bull()
                state = 3

            # If the bull's aruco is detected, change state to face the bull, the NAO should rotate towards the bull
            if (state == 2 or state == 3) and vision.bull_detected_top:
                state = 3
                state_pub.publish(state)
                motion.action('front')
                walking.rotate(vision.bull_angle * np.pi / 180)
                time.sleep(0.5)
                motion.action('down')

            # If the Nao faces the bull, she has to speak the phrase "Preparado" and wait for the operators command "Adelante"
            # and then challenge the bull with its leds and hold the cape like a matador
            if (state == 3 or state == 4) and np.abs(vision.bull_angle) <= 6 and vision.facing:
                state = 4
                state_pub.publish(state)
                time.sleep(1)
                speech.speak("Preparado")
                while state == 4 and not speech.heard:
                    pass
                time.sleep(1)
                motion.action('start')
                time.sleep(0.7)
                motion.action('challenge')
                while state == 4 and not vision.challenged:
                    rospy.loginfo('Attack me!')
                    leds.blink('challenge')
                    time.sleep(2)
                    if bumpers.hit:
                        break

            # If the Bull was challenged, the NAO must prepare and move to a side, rotate slightly and put the cape to a side
            if state == 4 and vision.challenged:
                state = 5
                state_pub.publish(state)
                walking.sideways(-0.20)
                time.sleep(2.5)
                walking.rotate(30 * np.pi /180)
                time.sleep(2)
                motion.action('move')
                time.sleep(0.5)

            # If the bull is too close to the NAO, it must juke the incoming enemy
            if state == 5 and vision.bull_distance <= np.abs(0.5):
                state = 6
                state_pub.publish(state)
                speech.play_ole()
                motion.action('juke')

                # If the bull does not hit the bumpers for 3 sec, then assume that the Juke was successful, play the "Hurra" routines
                temp = bumpers.hit
                for i in range(0, 299):
                    if temp:
                        break
                    time.sleep(0.01)
                    temp = bumpers.hit
                if state == 6 and not temp:
                    state = 8
                    state_pub.publish(state)
                    nao_points = nao_points + 1
                    leds.blink('hurra')
                    motion.action('hurra')
                    speech.points(nao_points, bull_points)
                    state = 2
                    state_pub.publish(state)

            # If the bumpers are ever hit, then the bull won, raise one point
            if bumpers.hit:
                state = 7
                state_pub.publish(state)
                speech.play_caramba()
                bull_points = bull_points + 1
                leds.blink('sad')
                motion.action('sad')
                speech.points(nao_points, bull_points)
                state = 2
                state_pub.publish(state)

        # Terminate the program
        except KeyboardInterrupt:
            state = -1
            state_pub.publish(state)
            rospy.logwarn('Corrida Terminated')


if __name__ == '__main__':
    """
        Main function of the program, it initializes the modules, and sets up the state publisher, then it starts the state machine
    """

    ###SETTING UP VARIABLES & ROS NODES
    rospy.init_node('corazon')
    # set up variables
    cur_state = 0
    rospy.logwarn("Started")

    ###SETTING UP THE PUBLISHERS & SETTING UP MODULES
    # Publishers & subscribers
    state_pub = rospy.Publisher('/state', msgs.Int8, queue_size=10)

    # Modules
    vision = VisionModule()
    walking = WalkingModule()
    leds = LedModule()
    bumpers = BumperModule()
    motion = MotionModule()
    speech = SpeechModule()

    ### STARTING THE STATE MACHINE
    cur_state = prep_machine(cur_state)
    rospy.logwarn("Going into the deep!")
    run_machine(cur_state)
