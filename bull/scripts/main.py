#!/usr/bin/env python
import rospy
from bull.msg import Tracking
from std_msgs.msg import Bool
import time
from Line_Tracking import *
from Motor import *
from servo import *
from Led import *
from Ultrasonic import *

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
led = Led()
ultrasonic=Ultrasonic()


def destroy():
	""" 
	This function will be executed if Ctrl+C is pressed
	"""
	print("Program was stopped!")
	wheel.setMotorModel(0,0,0,0)
	head.setServoPwm('0', 90)
	head.setServoPwm('1', 90)
	leds(off)

def leds(color):
	""" 
	Turn on LED based on color index
	"""
	led.ledIndex(0x01,color[0],color[1],color[2])
	led.ledIndex(0x02,color[0],color[1],color[2])
	led.ledIndex(0x04,color[0],color[1],color[2])
	led.ledIndex(0x08,color[0],color[1],color[2])
	led.ledIndex(0x10,color[0],color[1],color[2])
	led.ledIndex(0x20,color[0],color[1],color[2])
	led.ledIndex(0x40,color[0],color[1],color[2])
	led.ledIndex(0x80,color[0],color[1],color[2])

def go_to_border():
	""" 
	Find the border of arena before each round 
	"""
	## Move ahead until find the border
	while not rospy.is_shutdown() and GPIO.input(infrared.IR01)==False and GPIO.input(infrared.IR02)==False and GPIO.input(infrared.IR03)==False:
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
	"""
	Turn the head from left to right then right to left
	"""
	global old_x_angle, old_y_angle, red_detected
	print("Start looking for the matador.")
	y_angle = 105
	head.setServoPwm('1', y_angle)
	time.sleep(2)
	# Turn head to search
	for i in range(-10,180,1):
		head.setServoPwm('0',i)
		time.sleep(0.05)
		if red_detected:
			old_x_angle = i
			old_y_angle = y_angle
			return True
		if rospy.is_shutdown():
			break
	time.sleep(0.5)
	for i in range(180,-10,-1):
		head.setServoPwm('0',i)
		time.sleep(0.05)
		if red_detected: 
			old_x_angle = i
			old_y_angle = y_angle
			return True
		if rospy.is_shutdown():
			break
	time.sleep(0.5)
	return False

def head_tracking(offset_x, offset_y):
	""" 
	Object tracking based on pixel offset
	"""
	global old_x_angle, old_y_angle

	Kx = 0.4
	Ky = 0.2

	# Set minimum threshold
	if abs(offset_x) < offset_dead_block:
		offset_x = 0
	if abs(offset_y) < offset_dead_block:
		offset_y = 0

	delta_x = Kx * offset_x
	delta_y = - Ky * offset_y

	new_x_angle = old_x_angle + delta_x
	new_y_angle = old_y_angle + delta_y

	# Check angle range
	if new_x_angle < -10:
		new_x_angle = -10
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
	""" 
	Turn the wheel until head is pointing ahead and the whole car is facing NAO
	"""
	global old_x_angle, old_y_angle, red_offset_x, red_offset_y
	print('start.')
	print(old_x_angle)
	counter = 0
	while (old_x_angle > 95 or old_x_angle < 85) and not rospy.is_shutdown():
		# Stop the wheel every 10 iteration for head-tracking to catch up
		if counter == 10:
			wheel.setMotorModel(0,0,0,0)
			t0 = time.time()
			while time.time()-t0 < 0.8 and not rospy.is_shutdown():
				head_tracking(red_offset_x, red_offset_y)
			# time.sleep(0.5)
			counter = 0
			continue
		head_tracking(red_offset_x, red_offset_y)

		print(old_x_angle, old_y_angle)
		if old_x_angle > 95:
			wheel.setMotorModel(1500,800,-1500,-800)
		if old_x_angle < 85:
			wheel.setMotorModel(-1700,-800,1700,800)
		counter = counter + 1
	wheel.setMotorModel(800, 800, 800, 800)
	time.sleep(0.8)
	wheel.setMotorModel(0,0,0,0)
	print("Ready position reached.")


def charge():
	""" 
	Charge behavior
	"""
	wheel.setMotorModel(-800, -800, -800, -800)
	time.sleep(0.8)
	wheel.setMotorModel(800, 800, 800, 800)
	time.sleep(0.8)
	wheel.setMotorModel(-800, -800, -800, -800)
	time.sleep(0.8)
	## Linear trajectory
	wheel.setMotorModel(1000, 1000, 1000, 1000)
	time.sleep(3)
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

	old_x_angle = 90
	old_y_angle = 105
	head.setServoPwm('0', old_x_angle)
	head.setServoPwm('1', old_y_angle)
	leds(off)
	while not rospy.is_shutdown():
		
		#### Stage 1: Preparation
		leds(off)
		# Find the border
		go_to_border()
		
		### Circling the arena while searching for NAO
		# While not detected, circle for 1 sec then search again
		while not search_for_nao() and not rospy.is_shutdown():
			t0 = time.time()
			while not rospy.is_shutdown():
				circle.run()
				t1 = time.time()
				if (t1-t0 > 2):
					wheel.setMotorModel(0, 0, 0, 0)
					break

		print("****Matador detected.****")

		
		## Keep circling for a while to make sure the matador also sees the bull
		t0 = time.time()
		start = t0
		while not rospy.is_shutdown():
			circle.run()
			## Bull head tracks NAO
			head_tracking(red_offset_x, red_offset_y)

			t1 = time.time()
			if (t1-t0 > 1):
				wheel.setMotorModel(0, 0, 0, 0)
				time.sleep(3.5)
				t0 = time.time()
				continue
			if (t1-start > 25):
				wheel.setMotorModel(0, 0, 0, 0)
				print("****Let's fight.****")
				break
		
		if rospy.is_shutdown():
			break

		#### Stage 2: Go to ready position
		ready_position()
		

		#### Stage 3: LED communication
		# Light blue LED to tell NAO it's ready
		leds(blue)
		# Turn up the head to see in NAO's eye
		old_y_angle = 135
		head.setServoPwm('1', old_y_angle)
		# Polling for killer look
		while not challenged and not rospy.is_shutdown():
			pass

		print("NAO is challenging me")
		# Head returns to look front
		old_y_angle = 90
		head.setServoPwm('1', old_y_angle)
		# Light red LED to tell NAO it's attacking
		leds(red)
		time.sleep(15.5)
		

		if rospy.is_shutdown():
			break


		#### Stage 4: Charge        
		charge()
		time.sleep(5)

		# Detect if there's obstacle in front (if hit NAO)
		# If so, step back and turn away to find the border,
		# If not, ahead until find the border.
		distance = ultrasonic.get_distance()
		print ("NAO distance is "+str(distance)+"CM")
		if distance < 30:
			wheel.setMotorModel(-800, -800, -800, -800)
			time.sleep(1)
			wheel.setMotorModel(2000,2000,-1500,-1500)
			time.sleep(1)
			wheel.setMotorModel(0,0,0,0)



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
