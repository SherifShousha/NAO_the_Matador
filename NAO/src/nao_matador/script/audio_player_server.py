#!/usr/bin/env python

"""
    @author: Jaime Andrés González Eelman
    @author: Jesús Andrés Varona 
    @author: Sherif Shousha
    @author: Wenlan Shen 

    This server handles the reproduction of audio files, should they be requested via service.
"""

import rospy
import sys
from naoqi import ALProxy
from nao_matador.srv import Audio_msgs, Audio_msgsResponse

AudioPlayer = None


def audio_play(req):
    """
    Server callback, unpacks the sound requests and then compares each to play the right one
    :param req: service message
    :return: Bool, confirmation of play
    """

    sound = req.sound
    for aud in sound:
        rospy.loginfo(aud)
        if aud == "ole":
            fileId = AudioPlayer.loadFile("/home/nao/Ole.m4a")
            rospy.loginfo("Loaded")
            AudioPlayer.play(fileId)
            rospy.loginfo("Played")
            return Audio_msgsResponse(True)
        elif aud == 'caramba':
            fileId = AudioPlayer.loadFile("/home/nao/Caramba.m4a")
            AudioPlayer.play(fileId)
            return Audio_msgsResponse(True)


if __name__ == '__main__':
    """
    Main method called at execution, sets up the callback functions
    """

    robotIP = str(sys.argv[1])
    PORT = int(sys.argv[2])

    # Setting up the server
    AudioPlayer = ALProxy("ALAudioPlayer", robotIP, PORT)
    rospy.init_node('audio_server')
    rospy.Service('/sound_generator', Audio_msgs, audio_play)

    rospy.spin()
