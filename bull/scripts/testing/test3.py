#!/usr/bin/env python
import rospy
from bull.msg import Tracking
from std_msgs.msg import Bool
import time
from Line_Tracking import *
from Motor import *
from servo import *
# from Led import *


nao_detected = False
red_detected = False
challenged = False
offset_dead_block = 0.1

red = (255, 0, 0)
blue = (0, 0, 255)
green = (0, 255, 0)
off = (0, 0, 0)

# Initialize module objects
circle = infrared_Tracking()
wheel = Motor()
head = Servo()
# led = Led()


def destroy():
	print("Program was stopped!")
	wheel.setMotorModel(0,0,0,0)
	head.setServoPwm('0', 90)
	head.setServoPwm('1', 90)
	# leds(off)

def leds(color):
	led.ledIndex(0x01,color[0],color[1],color[2])
	led.ledIndex(0x02,color[0],color[1],color[2])
	led.ledIndex(0x04,color[0],color[1],color[2])
	led.ledIndex(0x08,color[0],color[1],color[2])
	led.ledIndex(0x10,color[0],color[1],color[2])
	led.ledIndex(0x20,color[0],color[1],color[2])
	led.ledIndex(0x40,color[0],color[1],color[2])
	led.ledIndex(0x80,color[0],color[1],color[2])

def go_to_border():
	## Move ahead until find the border
	if GPIO.input(infrared.IR01)==False and GPIO.input(infrared.IR02)==False and GPIO.input(infrared.IR03)==False:
		wheel.setMotorModel(800,800,800,800)
	# Follow the line for 2 sec then stop
	t0 = time.time()
	while not rospy.is_shutdown():
		infrared.run()
		t1 = time.time()
		if (t1-t0) > 2:
			wheel.setMotorModel(0, 0, 0, 0)
			break

def search_for_nao():
	global old_x_angle, old_y_angle, nao_detected
	print("Start looking for the matador.")
	y_angle = 105
	head.setServoPwm('1', y_angle)
	while(red_detected == False) and (not rospy.is_shutdown()):
		# Turn head to search
		for i in range(-10,180,1):
			head.setServoPwm('0',i)
			time.sleep(0.05)
			if nao_detected or rospy.is_shutdown():
				break
		if nao_detected or rospy.is_shutdown():
			break
		time.sleep(0.1)
		for i in range(180,-10,-1):
			head.setServoPwm('0',i)
			time.sleep(0.05)
			if nao_detected or rospy.is_shutdown():
				break
	if nao_detected:
		# Save servo position
		old_x_angle = i
		old_y_angle = y_angle
		print("****Matador detected.****")

def head_tracking(offset_x, offset_y):
	global old_x_angle, old_y_angle

	Kx = 0.5
	Ky = 0.2

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

def ready_position():
	global old_x_angle, old_y_angle, red_offset_x, red_offset_y

	print('start.')
	print(old_x_angle)
	# while not nao_detected:
	# 	pass
	counter = 0
	while (old_x_angle > 95 or old_x_angle < 85) and not rospy.is_shutdown():
		if counter == 20:
			wheel.setMotorModel(0,0,0,0)
			t0 = time.time()
			while time.time()-t0 < 0.5 and not rospy.is_shutdown():
				head_tracking(red_offset_x, red_offset_y)
			# time.sleep(0.5)
			counter = 0
			continue
		head_tracking(red_offset_x, red_offset_y)
		# time.sleep(0.1)
		print(old_x_angle, old_y_angle)
		if old_x_angle > 95:
			wheel.setMotorModel(2000,800,-2000,-800)
			# time.sleep(0.1)
			# print ("The car is turning right")
		if old_x_angle < 85:
			wheel.setMotorModel(-2000,-800,2000,800)
			# time.sleep(0.1)      
			# print ("The car is turning left")
		counter = counter + 1
	print('finished.')
	wheel.setMotorModel(0,0,0,0)



def prepare_to_charge():
	global old_x_angle, old_y_angle, nao_offset_x, nao_offset_y
	## Track the red cloth
	# while not rospy.is_shutdown():
	# 	print ("Go to the ready position")
	# 	# if red_detected == False:
	# 	# 	break
	# 	if red_offset_x < -0.1:
	# 		print(red_offset_x)
	# 		wheel.setMotorModel(-1500, -1500, 2000, 2000)
	# 		time.sleep(0.05)
	# 	elif red_offset_x > 0.1:
	# 		print(red_offset_x)
	# 		wheel.setMotorModel(2000, 2000, -1500, -1500)
	# 		time.sleep(0.05)
	# 	else:
	# 		wheel.setMotorModel(600, 600, 600, 600)
	# 		time.sleep(0.1)
	# 		wheel.setMotorModel(0, 0, 0, 0)
	# 		break

	while (old_x_angle > 95 or old_x_angle < 85) and rospy.is_shutdown():
		head_tracking(nao_offset_x, nao_offset_y)
		if old_x_angle > 95:
			wheel.setMotorModel(4000,4000,-2000,-2000) 
			time.sleep(0.1)       
			print ("The car is turning right")
		if old_x_angle < 85:
			wheel.setMotorModel(-2000,-2000,4000,4000)        
			print ("The car is turning left")
			time.slepp(0.1)
	wheel.setMotorModel(0,0,0,0)
	
	# # print(red_offset_x)
	# time.sleep(1)
	# if red_detected:
	# 	print ("Ready position reached.")
	# 	time.sleep(1)
	# 	wheel.setMotorModel(1000, 1000, 1000, 1000)
	# 	time.sleep(0.5)
	# 	wheel.setMotorModel(0, 0, 0, 0)
	# else:
	# 	print ("Lost track of red cloth.")

def charge():
	## Linear trajectory
	wheel.setMotorModel(1500, 1500, 1500, 1500)
	time.sleep(1.0)
	wheel.setMotorModel(0, 0, 0, 0)


##########################################################
#### Callback functions

def challenge_callback(input):
	global challenged
	challenged = input.data

def nao_tracking_callback(nao_tracking_msg): 
	global nao_offset_x, nao_offset_y, nao_detected
	nao_detected = nao_tracking_msg.detected
	nao_offset_x = nao_tracking_msg.x
	nao_offset_y = nao_tracking_msg.y

def red_tracking_callback(red_tracking_msg):
	# subscribe 
	global red_offset_x, red_offset_y, red_detected
	red_detected = red_tracking_msg.detected
	red_offset_x = red_tracking_msg.x
	red_offset_y = red_tracking_msg.y
###########################################################

def main_loop():
	global red_detected, challenged
	global nao_offset_x, nao_offset_y
	global old_x_angle, old_y_angle

	old_x_angle = 180
	old_y_angle = 105
	head.setServoPwm('0', old_x_angle)
	head.setServoPwm('1', old_y_angle)
	time.sleep(2)

	ready_position()

	# while not rospy.is_shutdown():
		

	# 	#### Stage 1: Preparation
	# 	leds(red)
		# Find the border
		# go_to_border()
		# # Turn its head to search for NAO
		# search_for_nao()
		
		# # if detected, start circling around the arena
		# t0 = time.time()
		# while not rospy.is_shutdown():
		# 	## Car circles around the arena
		# 	circle.run()
		# 	## Bull head tracks NAO
		# 	head_tracking(nao_offset_x, nao_offset_y)

		# 	## If red cloth detected, stop circling
		# 	t1 = time.time()
		# 	if (t1-t0 > 10):
		# 		leds(off)
		# 		print("****Let's fight.****")
		# 		# time.sleep(0.2)
		# 		PWM.setMotorModel(0, 0, 0, 0)
		# 		break   
		
		# # #### Stage 2: Go to ready position
		# ready_position()
		# break
		# print("Ready position reached.")
		# leds(blue)

		# # #### Stage 3:
		# while not challenged and not rospy.is_shutdown():
		# 	pass
		# leds(red)
		# time.sleep(3)
		# prepare_to_charge()

		# # #### Stage 4: Charge        
		# charge()
		# time.sleep(3)

if __name__ == '__main__':

	# ROS initialization
	rospy.init_node('bull_node', anonymous=True)
	rospy.Subscriber('/nao_tracking', Tracking, nao_tracking_callback)
	rospy.Subscriber('/red_tracking', Tracking, red_tracking_callback)
	rospy.Subscriber('/nao_challenge', Bool, challenge_callback)

	main_loop()        
	rospy.spin()

	# If Ctrl+ C is pressed, destroy everything
	rospy.on_shutdown(destroy)
