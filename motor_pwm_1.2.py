# Ember Ipek - TETRA team 11/2/2025
# Obstacle avoidance for multiple motors using HC-SR04 ultrasonic sensor
# to control PWM duty cycle
# implemented button control 11/6/2025
#
# 11/29/2025
# modified for use with 30W motors & one driver, MPU gyroscope functionality implemented

import RPi.GPIO as GPIO
import time
import numpy as np
import smbus


GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

triggerPin = 7
echoPin = 11

button0 = 40
button1 = 38
button2 = 36

# MOSFETs control which motor is enabled
# motor 1 always ON, refer to crappy drawing
motor_cntl = [32, 37, 31]

PWMScalar = 1
maxDist = 75

# motors = [[13, 15], [16, 18], [22, 29]]

# these are now just the PWM inputs to the driver
motors = [13, 15]
motorsPWM = []

# The slave address of the MPU-60X0 is b110100X which is 7 bits long. The LSB bit of the 7 bit address is
# determined by the logic level on pin AD0
mpu_address = 0x68
bus = smbus.SMBus(1)

# pin 3 SDA, pin 5 SCL
# SDA and SCL lines typically need pull-up resistors to VDD

def PWMSetup():
	for i in motors:
		GPIO.setup(i, GPIO.OUT)
		#PWM freq = 2kHz
		pwm = GPIO.PWM(i, 2000)
		pwm.start(0)
		motorsPWM.append(pwm)
	
	return

def buttonSetup():
	# resistors required for button debounce
	GPIO.setup(button0, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
	GPIO.setup(button1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
	GPIO.setup(button2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
	GPIO.setup(motor_cntl[0], GPIO.OUT, initial=GPIO.LOW)
	GPIO.setup(motor_cntl[1], GPIO.OUT, initial=GPIO.LOW)
	GPIO.setup(motor_cntl[2], GPIO.OUT, initial=GPIO.LOW)

	return

def checkdist():
	GPIO.output(triggerPin, GPIO.HIGH)
	time.sleep(0.000015)
	GPIO.output(triggerPin, GPIO.LOW)
	
	#timout check so function doesn't stall
	timeout = time.time() + 0.02
	while not GPIO.input(echoPin):
		if time.time() > timeout:
			return maxDist
	# while not GPIO.input(echoPin):
	# 	pass
	t1 = time.time()
	timeout = time.time() + 0.02
	while GPIO.input(echoPin):
		if time.time() > timeout:
			return maxDist
	# while GPIO.input(echoPin):
	# 	pass
	t2 = time.time()
	# print("t1: ", t1, "t2: ", t2)
	
	return (t2 - t1) * (340 / 2) * 100

# normalize distance to a percentage of given range
def normalizedDistance(dist, maxDist, minDist):
	if maxDist <= minDist:
		# shouldn't reach here if user is not stupid
		return 0
	dist = dist - minDist
	maxDist = maxDist - minDist
	normalized = max(0, min(dist / maxDist, 1))
	
	return normalized

def normalizedPWM(dist, maxDist=75, minDist=10):
	# distance controls PWM percentage
	duty = 100 * normalizedDistance(dist, maxDist, minDist)
	# use first pin for forward PWM and keep the other side 0
	motorsPWM[0].ChangeDutyCycle(duty)
	motorsPWM[1].ChangeDutyCycle(0)
	
	return

def motorRun(motorNum, status, direction=1):
	if(status == 1):
		if(direction == 1):
			# GPIO.output(motors[motorNum][0], GPIO.HIGH)
			# GPIO.output(motors[motorNum][1], GPIO.LOW)
			dist = checkdist()
			normalizedPWM(dist)
			# motorNum is ON
			GPIO.output(motor_cntl[motorNum], GPIO.HIGH)

			#print("MOTOR NUM ", motorNum, "PWM ", 100 * normalizedDistance(dist, maxDist, 0), "%\n")
		else:
			# GPIO.output(motors[motorNum][0], GPIO.LOW)
			# GPIO.output(motors[motorNum][1], GPIO.HIGH)
			dist = checkdist()
			normalizedPWM(dist)
			# motorNum is ON
			GPIO.output(motor_cntl[motorNum], GPIO.HIGH)

			#print("MOTOR NUM ", motorNum, "PWM ", 100 * normalizedDistance(dist, maxDist, 0), "%\n")
	else:
		# GPIO.output(motors[motorNum][0], GPIO.HIGH)
		# GPIO.output(motors[motorNum][1], GPIO.HIGH)

		# motorNum OFF
		GPIO.output(motor_cntl[motorNum], GPIO.LOW)

		#print("MOTOR NUM ", motorNum, "STOPPED\n")
	return

def loop():
	try:
		button0_state = False
		button1_state = False
		button2_state = False

		motor0_state = False
		motor1_state = False
		motor2_state = False

		while(True):
			# loop motors
			dist = checkdist()
			PWMScalar = normalizedDistance(dist, maxDist, 0)

			#print("distance: ", dist, "\n")
			#print("PWMScalar: ", PWMScalar, "\n")

			# latch to avoid multiple inputs while holding down button
			# button 0: face A   button 1: face B   button 2: face C
			# A: motors 0, 1	 B: motors 1, 3		C: motors 1, 2
			button0_pressed = GPIO.input(button0) == 1
			button1_pressed = GPIO.input(button1) == 1
			button2_pressed = GPIO.input(button2) == 1
   
			#set motor states
			if(button0_pressed and not button0_state):
				if(motor0_state):
					motor0_state = False
				else:
					motor0_state = True
			button0_state = button0_pressed

			if(button1_pressed and not button1_state):
				if(motor1_state):
					motor1_state = False
				else:
					motor1_state = True
			button1_state = button1_pressed

			if(button2_pressed and not button2_state):
				if(motor2_state):
					motor2_state = False
				else:
					motor2_state = True
			button2_state = button2_pressed

			# motorRun(0, 1)
			# motorRun(1, 1)
			# motorRun(2, 1)
			if(motor0_state):
				motorRun(0, 1)
			else:
				motorRun(0, 0)
			time.sleep(0.02)

			if(motor1_state):
				motorRun(1, 1)
			else:
				motorRun(1, 0)
			time.sleep(0.02)

			if(motor2_state):
				motorRun(2, 1)
			else:
				motorRun(2, 0)
			time.sleep(0.02)

			mpu_val_x = bus.read_i2c_block_data(mpu_address, 0x43, 2)
			mpu_val_y = bus.read_i2c_block_data(mpu_address, 0x45, 2)
			mpu_val_z = bus.read_i2c_block_data(mpu_address, 0x47, 2)
			print("Gyroscope X: ", mpu_val_x, ", Y: ", mpu_val_y, ", Z: ", mpu_val_z)
			# time.sleep(0.5)

	except KeyboardInterrupt:
		GPIO.cleanup()
	return

GPIO.setup(triggerPin, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(echoPin, GPIO.IN)

buttonSetup()

PWMSetup()
# reg 0x6B: power management, clear all bits to wake up mpu
bus.write_byte_data(mpu_address, 0x6B, 0)

loop()