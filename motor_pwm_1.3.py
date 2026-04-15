# Ember Ipek - TETRA team 11/2/2025
# Obstacle avoidance for multiple motors using HC-SR04 ultrasonic sensor
# to control PWM duty cycle
# implemented button control 11/6/2025
#
# 11/29/2025
# modified for use with 30W motors & one driver, MPU gyroscope functionality implemented
# 4/10/2026
# modified for use with 4 motor controllers and individual RPWM and LPWM controls
# button testing functionality removed, move based on sensors now, not button inputs

import RPi.GPIO as GPIO
import time
import numpy as np
import smbus


GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

triggerPin = 7
echoPin = 11

# motor 0:
#	LPWM: pin 13
#	RPWM: pin 15

# motor 1:
#	LPWM: pin 12
#	RPWM: pin 18

# motor 2:
#	LPWM: pin 29
#	RPWM: pin 31

# motor 3:
#	LPWM: pin 33
#	RPWM: pin 37
# motor 1 always ON
motor_cntl = [[13, 15], [12, 18], [29, 31], [33, 37]]
LPWM = 0
RPWM = 1
face_driving = ['A', 'B', 'C', 'D']

PWMScalar = 1
maxDist = 75

motorsPWM = []

# The slave address of the MPU-60X0 is b110100X which is 7 bits long.
# The LSB bit of the 7 bit address is determined by the logic level on pin AD0
mpu_address = 0x68
bus = smbus.SMBus(1)

# pin 3 SDA, pin 5 SCL
# SDA and SCL lines typically need pull-up resistors to VDD

def PWMSetup():
	# for(int i = 0, i < motor_cnt.len; i++)
	for i in range(len(motor_cntl)):
		PWM_in = []
		# for(int j = 0, j < motor_cntl[i.len]; j++){
		for j in range(len(motor_cntl[i])):
			pin = motor_cntl[i][j]
			GPIO.setup(pin, GPIO.OUT)
			#PWM freq = 2kHz
			pwm = GPIO.PWM(pin, 2000)
			pwm.start(0)
			PWM_in.append(pwm)
		motorsPWM.append(PWM_in)
	
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
def normalizedDistance(dist, maxDist=75, minDist=10):
	if maxDist <= minDist:
		# shouldn't reach here if user is not stupid
		return 0
	dist = dist - minDist
	maxDist = maxDist - minDist
	normalized = max(0, min(dist / maxDist, 1))
	
	return normalized

# implements collision avoidance
# args: distance measured by ultrasonic sensor, maxDist, minDist
# returns a duty cycle based on dist
def normalizedPWM(dist, maxDist=75, minDist=10):
	# distance controls PWM percentage
	dutyCycle = 100 * normalizedDistance(dist, maxDist, minDist)
	# motor spinning way too fast, use reasonable speed for testing
	dutyCycle *= 0.2
	# use drive functions to change PWM now
	# motorsPWM[0].ChangeDutyCycle(duty)
	# motorsPWM[1].ChangeDutyCycle(0)
	
	return dutyCycle


# turn right A: motor 0 right, motor 1 right, motor 2 right
def turnRightFaceA():
	dist = checkdist()
	dutyCycle = normalizedPWM(dist)
	
	# turn very slowly for now.
	motorsPWM[0][LPWM].ChangeDutyCycle(0)
	motorsPWM[0][RPWM].ChangeDutyCycle(5)
	
	motorsPWM[1][LPWM].ChangeDutyCycle(0)
	motorsPWM[1][RPWM].ChangeDutyCycle(5)
	
	motorsPWM[2][LPWM].ChangeDutyCycle(0)
	motorsPWM[2][RPWM].ChangeDutyCycle(5)
	
	motorsPWM[3][LPWM].ChangeDutyCycle(100)
	motorsPWM[3][RPWM].ChangeDutyCycle(100)
	
	return

# make helper functions to go forward or reverse, maybe take face driving as args??
def forwardFaceA():
	# motor 0 RPWM, motor 1 LPWM
	# brake other motors
	dist = checkdist()
	dutyCycle = normalizedPWM(dist)
	
	motorsPWM[0][LPWM].ChangeDutyCycle(0)
	motorsPWM[0][RPWM].ChangeDutyCycle(dutyCycle)
	
	motorsPWM[1][LPWM].ChangeDutyCycle(dutyCycle)
	motorsPWM[1][RPWM].ChangeDutyCycle(0)
	
	motorsPWM[2][LPWM].ChangeDutyCycle(100)
	motorsPWM[2][RPWM].ChangeDutyCycle(100)
	
	motorsPWM[3][LPWM].ChangeDutyCycle(100)
	motorsPWM[3][RPWM].ChangeDutyCycle(100)

	PWMScalar = normalizedDistance(dist)

	print("distance: ", dist, "\n")
	print("PWMScalar: ", PWMScalar, "\n")
	
	return

# turn right B: motor 0 right, motor 3 right, motor 1 right
def turnRightFaceB():
	dist = checkdist()
	dutyCycle = normalizedPWM(dist)
	
	# turn very slowly for now.
	motorsPWM[0][LPWM].ChangeDutyCycle(0)
	motorsPWM[0][RPWM].ChangeDutyCycle(5)
	
	motorsPWM[3][LPWM].ChangeDutyCycle(0)
	motorsPWM[3][RPWM].ChangeDutyCycle(5)
	
	motorsPWM[1][LPWM].ChangeDutyCycle(0)
	motorsPWM[1][RPWM].ChangeDutyCycle(5)
	
	motorsPWM[2][LPWM].ChangeDutyCycle(100)
	motorsPWM[2][RPWM].ChangeDutyCycle(100)
	
	return

# forward B: motor 0 right, motor 3 left
def forwardFaceB():
	dist = checkdist()
	dutyCycle = normalizedPWM(dist)
	
	motorsPWM[0][LPWM].ChangeDutyCycle(0)
	motorsPWM[0][RPWM].ChangeDutyCycle(dutyCycle)
	
	motorsPWM[3][LPWM].ChangeDutyCycle(dutyCycle)
	motorsPWM[3][RPWM].ChangeDutyCycle(0)
	
	motorsPWM[2][LPWM].ChangeDutyCycle(100)
	motorsPWM[2][RPWM].ChangeDutyCycle(100)
	
	motorsPWM[1][LPWM].ChangeDutyCycle(100)
	motorsPWM[1][RPWM].ChangeDutyCycle(100)
	
	return

# turn right C: motor 0 right, motor 2 right, motor 3 right
def turnRightFaceC():
	dist = checkdist()
	dutyCycle = normalizedPWM(dist)
	
	# turn very slowly for now.
	motorsPWM[0][LPWM].ChangeDutyCycle(0)
	motorsPWM[0][RPWM].ChangeDutyCycle(5)
	
	motorsPWM[2][LPWM].ChangeDutyCycle(0)
	motorsPWM[2][RPWM].ChangeDutyCycle(5)
	
	motorsPWM[3][LPWM].ChangeDutyCycle(0)
	motorsPWM[3][RPWM].ChangeDutyCycle(5)
	
	motorsPWM[1][LPWM].ChangeDutyCycle(100)
	motorsPWM[1][RPWM].ChangeDutyCycle(100)
	
	return

# forward C: motor 0 right, motor 2 left
def forwardFaceC():
	dist = checkdist()
	dutyCycle = normalizedPWM(dist)
	
	motorsPWM[0][LPWM].ChangeDutyCycle(0)
	motorsPWM[0][RPWM].ChangeDutyCycle(dutyCycle)
	
	motorsPWM[2][LPWM].ChangeDutyCycle(dutyCycle)
	motorsPWM[2][RPWM].ChangeDutyCycle(0)
	
	motorsPWM[3][LPWM].ChangeDutyCycle(100)
	motorsPWM[3][RPWM].ChangeDutyCycle(100)
	
	motorsPWM[1][LPWM].ChangeDutyCycle(100)
	motorsPWM[1][RPWM].ChangeDutyCycle(100)
	
	return

def brake():
	motorsPWM[0][LPWM].ChangeDutyCycle(100)
	motorsPWM[0][RPWM].ChangeDutyCycle(100)
	motorsPWM[1][LPWM].ChangeDutyCycle(100)
	motorsPWM[1][RPWM].ChangeDutyCycle(100)
	motorsPWM[2][LPWM].ChangeDutyCycle(100)
	motorsPWM[2][RPWM].ChangeDutyCycle(100)
	motorsPWM[3][LPWM].ChangeDutyCycle(100)
	motorsPWM[3][RPWM].ChangeDutyCycle(100)
	
	return

# A -> B -> C -> back to A

def loop():
	try:

		while(True):
			# loop motors
			# dist = checkdist()
			# PWMScalar = normalizedDistance(dist, maxDist, 0)

			# print("distance: ", dist, "\n")
			# print("PWMScalar: ", PWMScalar, "\n")

			# todo: implement face detection logic.
			# flipped = (new_face != old_face);
			# if(flipped){
			#	todo: turn right to re-orient face when flipped.
			# 	switch(face_driving){
			# 		case(A): turnRightA();
			#				 driveFaceA(); ... etc etc

			# drive a bit
			forwardFaceA()
			time.sleep(.5)
			# spin around for a bit
			turnRightFaceA()
			time.sleep(2)
			
			# gyroscope values are 2 bytes each axis, stored in regs 0x43-0x49 in 2s compliment format
			# convert to decimal and find out which face corresponds to values
			# but first, make helper function to read the values getGyroX() getGyroY() getGyroZ()
			# if((something > val_x > something) && (something > val_y > something) && (something > val_z > something &&))
			# 	face_driving = A
			# IMPORTANT: GYROSCOPE POSITION MUST BE STABLE RELATIVE TO CHASSIS FOR ACCURATE READINGS
			mpu_val_x = bus.read_i2c_block_data(mpu_address, 0x43, 2)
			mpu_val_y = bus.read_i2c_block_data(mpu_address, 0x45, 2)
			mpu_val_z = bus.read_i2c_block_data(mpu_address, 0x47, 2)
			print("Gyroscope X: ", mpu_val_x, ", Y: ", mpu_val_y, ", Z: ", mpu_val_z)

			time.sleep(0.1)

	except KeyboardInterrupt:
		GPIO.cleanup()
	return

GPIO.setup(triggerPin, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(echoPin, GPIO.IN)

# buttonSetup()

PWMSetup()
# reg 0x6B: power management, clear all bits to wake up mpu
bus.write_byte_data(mpu_address, 0x6B, 0)

try:
	loop()
finally:
	GPIO.cleanup()