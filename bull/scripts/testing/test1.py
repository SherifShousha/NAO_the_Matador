#!/usr/bin/env python
""" Test for turning its head to search for NAO (in this file replaced with red_searching for test reason) and arena circling"""

import rospy
from bull.msg import Tracking
import time
from Line_Tracking import *
from Motor import *
from servo import *


# Initialize module objects
wheel = Motor()
head = Servo()


red_detected = False
nao_detected = False
offset_dead_block = 0.1

def destroy():
	print("Program was stopped!")
	wheel.setMotorModel(0,0,0,0)
	head.setServoPwm('0', 90)
	head.setServoPwm('1', 110)


def search_for_boarder():
	# Move ahead until find the boarder
	if GPIO.input(infrared.IR01)==False and GPIO.input(infrared.IR02)==False and GPIO.input(infrared.IR03)==False:
		PWM.setMotorModel(800,800,800,800)
	# Follow the line for 2 sec then stop
	t0 = time.time()
	while(True):
		infrared.run()
		t1 = time.time()
		if (t1-t0) > 2:
			wheel.setMotorModel(0, 0, 0, 0)
			break


def search_for_nao():
	global old_x_angle, old_y_angle, nao_detected
	print("Start serching for the matador.")
	head.setServoPwm('1',110)
	while(not nao_detected) and (not rospy.is_shutdown()):
		# Turn head to search
		for i in range(-10,180,1):
			head.setServoPwm('0',i)
			time.sleep(0.05)
			if nao_detected or rospy.is_shutdown():
				break
		if nao_detected or rospy.is_shutdown():
			break
		for i in range(180,-10,-1):
			head.setServoPwm('0',i)
			time.sleep(0.05)
			if red_detected or rospy.is_shutdown():
				break
	if nao_detected:
		# Save servo position
		old_x_angle = i
		old_y_angle = 110
		print("****Matador detected.****")


def head_tracking(offset_x, offset_y):
	global old_x_angle, old_y_angle

	Kx = 0.5
	Ky = 0.5

	# Set minimum threshold
	if abs(offset_x) < offset_dead_block:
		offset_x = 0
	if abs(offset_y) < offset_dead_block:
		offset_y = 0

	delta_x = Kx * offset_x
	delta_y = -Ky * offset_y
	
	new_x_angle = old_x_angle + delta_x
	new_y_angle = old_y_angle + delta_y
	
	# Check angle range
	if new_x_angle < -30:
		new_x_angle = -30
	if new_x_angle > 180:
		new_x_angle = 180
	if new_y_angle < 60:
		new_y_angle = 60
	if new_y_angle > 150:
		new_y_angle = 150

	# print("X angle: {}, Y angle:{}".format(int(new_x_angle), int(new_y_angle)))
	
	head.setServoPwm('0', int(new_x_angle))
	head.setServoPwm('1', int(new_y_angle))

	old_x_angle = new_x_angle
	old_y_angle = new_y_angle

def nao_tracking_callback(nao_tracking_msg): 
    global nao_offset_x, nao_offset_y, nao_detected
    nao_detected = nao_tracking_msg.detected
    nao_offset_x = nao_tracking_msg.x
    nao_offset_y = nao_tracking_msg.y

def red_tracking_callback(red_tracking_msg): 
	global red_offset_x, red_offset_y, red_detected
	red_detected = red_tracking_msg.detected
	red_offset_x = red_tracking_msg.x
	red_offset_y = red_tracking_msg.y

def main_loop():
	global red_offset_x, red_offset_y

	# search_for_boarder()
	
	search_for_nao()
	
	# if serched, start circle around the arena
	while not rospy.is_shutdown():
		## Car circles around the arena
		# infrared.run()
		## Bull head tracks NAO
		head_tracking(nao_offset_x, nao_offset_y)

   

if __name__ == '__main__':

	# ROS initialization
	rospy.init_node('bull_node', anonymous=True)
	rospy.Subscriber('/nao_tracking', Tracking, nao_tracking_callback)
	rospy.Subscriber('/red_tracking', Tracking, red_tracking_callback)

	main_loop()		
	rospy.spin()
	# If Ctrl+ C is pressed, destroy everything
	rospy.on_shutdown(destroy)