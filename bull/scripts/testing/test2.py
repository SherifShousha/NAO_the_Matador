#!/usr/bin/env python
""" Test for go to ready position """

import rospy
from bull.msg import Tracking
import time
from Line_Tracking import *
from Motor import *
from servo import *


# Initialize module objects
circle = infrared_Tracking()
wheel = Motor()
head = Servo()
red_detected = False
direction = -1

def destroy():
	print("Program was stopped!")
	wheel.setMotorModel(0,0,0,0)
	head.setServoPwm('0', 90)
	head.setServoPwm('1', 90)

def ready_position():
	global old_x_angle, old_y_angle, red_offset_x
	## Head turns back to front
	head.setServoPwm('0', 90)
	head.setServoPwm('1', 105)
	## Car moves to right in front of NAO
	if old_x_angle < 70:    # NAO is on the left side
		# turn left 
		wheel.setMotorModel(-1500,-1500,2000,2000)        
		print ("The car is turning left")
		time.sleep(1.5)
		wheel.setMotorModel(0,0,0,0)
	elif old_x_angle > 110:     # NAO is on the right side
		# turn right
		wheel.setMotorModel(2000,2000,-1500,-1500)        
		print ("The car is turning right")
		time.sleep(1)
		wheel.setMotorModel(0,0,0,0)
	time.sleep(2.0)


	## TODO: The current method is unstable, replace this with a proportional controller
	# while not rospy.is_shutdown():
	# 	print ("Go to the ready position")
	# 	if red_detected == False:
	# 		break
	# 	if red_offset_x < -0.1:
	# 		print(red_offset_x)
	# 		wheel.setMotorModel(-600, -600, 1000, 1000)
	# 		time.sleep(0.05)
	# 	elif red_offset_x > 0.1:
	# 		print(red_offset_x)
	# 		wheel.setMotorModel(1000, 1000, -600, -600)
	# 		time.sleep(0.05)
	# 	else:
	# 		wheel.setMotorModel(600, 600, 600, 600)
	# 		time.sleep(0.1)
	# 		wheel.setMotorModel(0, 0, 0, 0)
	# 		time.sleep(1)
	# 		break

	# print(red_offset_x)
	# # wheel.setMotorModel(800, 800, 800, 800)
	# while not red_detected and not rospy.is_shutdown():
	# 	pass
	# print ("Ready position reached.")
	# time.sleep(1)
	# wheel.setMotorModel(1000, 1000, 1000, 1000)
	# time.sleep(2.5)
	# wheel.setMotorModel(0, 0, 0, 0)



def red_tracking_callback(red_tracking_msg): 
	global red_offset_x, red_offset_y, red_detected
	red_detected = red_tracking_msg.detected
	red_offset_x = red_tracking_msg.x
	red_offset_y = red_tracking_msg.y
	

def main_loop():
	global red_offset_x, red_offset_y
	global old_x_angle, old_y_angle
	# TODO: turn bull head
	# search_for_nao()
	old_x_angle = 0
	old_y_angle = 105
	head.setServoPwm('0', old_x_angle)
	head.setServoPwm('1', old_y_angle)
	# if serched, start circle around the arena
	# while not rospy.is_shutdown():
		

		# Car circles around the arena
		# print("****Circling****")
		# circle.run()
		# # Bull head tracks NAO
		# # head_tracking(red_offset_x, red_offset_y)
		# # If red cloth detected
		# if (red_detected):
		# 	# # continue to circle boarder for 1 more sec then stop
		# 	# t0 = time.time()
		# 	# while not rospy.is_shutdown():
		# 	# 	t1 = time.time()
		# 	# 	circle.run()
		# 	# 	if (t1-t0) > 0.5:
		# 	# 		wheel.setMotorModel(0, 0, 0, 0)
		# 	# 		break
		# 	wheel.setMotorModel(0, 0, 0, 0)
		# 	print("****Red cloth detected.****")
		# 	break
		
   	#### Stage 2: Go to ready position
	### Tasks:
	
	ready_position()


if __name__ == '__main__':

	# ROS initialization
	rospy.init_node('bull_node', anonymous=True)
	# rospy.Subscriber('/nao_tracking', Tracking, nao_tracking_callback)
	rospy.Subscriber('/red_tracking', Tracking, red_tracking_callback)
	time.sleep(0.5)
	main_loop()		
	rospy.spin()
	# If Ctrl+ C is pressed, destroy everything
	rospy.on_shutdown(destroy)