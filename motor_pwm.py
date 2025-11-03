# Ember Ipek - TETRA team 11/2/2025
# Obstacle avoidance for multiple motors using HC-SR04 ultrasonic sensor
# to control PWM duty cycle

import RPi.GPIO as GPIO
import time
import numpy as np

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

triggerPin = 7
echoPin = 11

motorRows = 2
motorColumns = 2
PWMScalar = 1
maxDist = 50

motors = [[13, 15], [16,18]]
# motorsPWM = np.zeros((motorRows, motorColumns), dtype=GPIO.PWM)
# cannot use np.zeros, dtype defaults to float
motorsPWM = [[None for _ in range(motorColumns)] for _ in range(motorRows)]

def motorSetup(motorPin1, motorPin2):
	GPIO.setup(motorPin1, GPIO.OUT, initial=GPIO.LOW)
	GPIO.setup(motorPin2, GPIO.OUT, initial=GPIO.LOW)
	
	return

def PWMSetup():
	for i in range(motorRows): 
		for j in range(motorColumns):
			GPIO.setup(motors[i][j], GPIO.OUT)
			#PWM freq = 2kHz
			motorsPWM[i][j] = GPIO.PWM(motors[i][j], 2000)
			motorsPWM[i][j].start(0)
	
	return

def checkdist():
	GPIO.output(triggerPin, GPIO.HIGH)
	time.sleep(0.000015)
	GPIO.output(triggerPin, GPIO.LOW)
	
	# #timout check so function doesn't stall
	# timeout = time.time() + 0.02
	# while not GPIO.input(echoPin):
	# 	if time.time() > timeout:
	# 		return maxDist
	while not GPIO.input(echoPin):
		pass
	t1 = time.time()
	# timeout = time.time() + 0.02
	# while GPIO.input(echoPin):
	# 	if time.time() > timeout:
	# 		return maxDist
	while GPIO.input(echoPin):
		pass
	t2 = time.time()
	# print("t1: ", t1, "t2: ", t2)
	
	return (t2 - t1) * (340 / 2) * 100

# normalize distance to a percentage of given range
def normalizedDistance(dist, maxDist):
	normalized = max(0, min(dist / maxDist, 1))
	
	return normalized

def normalizedPWM(motorNum, dist, maxDist=50):
	# distance controls PWM percentage
	# motorsPWM[motorNum][0].ChangeDutyCycle(100 * normalizedDistance(dist, maxDist))
	duty = 100 * normalizedDistance(dist, maxDist)
	# use first pin for forward PWM and keep the other side 0
	motorsPWM[motorNum][0].ChangeDutyCycle(duty)
	motorsPWM[motorNum][1].ChangeDutyCycle(0)
	
	return

def motor(motorNum, status, direction=1):
	if(status == 1):
		if(direction == 1):
			# GPIO.output(motors[motorNum][0], GPIO.HIGH)
			# GPIO.output(motors[motorNum][1], GPIO.LOW)
			dist = checkdist()
			normalizedPWM(motorNum, dist)
			
			print("MOTOR NUM ", motorNum, "PWM 1\n")
		else:
			# GPIO.output(motors[motorNum][0], GPIO.LOW)
			# GPIO.output(motors[motorNum][1], GPIO.HIGH)
			dist = checkdist()
			normalizedPWM(motorNum, dist)
			print("MOTOR NUM ", motorNum, "PWM 0\n")
	else:
		# GPIO.output(motors[motorNum][0], GPIO.HIGH)
		# GPIO.output(motors[motorNum][1], GPIO.HIGH)
		motorsPWM[motorNum][0].ChangeDutyCycle(100)

		print("MOTOR NUM ", motorNum, "FULL SPEED\n")
	return

def loop():
	while(True):
		try:
			# loop motors
			dist = checkdist()
			PWMScalar = normalizedDistance(dist, maxDist)

			print("distance: ", dist, "\n")
			print("PWMScalar: ", PWMScalar, "\n")

			# motor(0, 1, 1)
			# motor(1, 1, 1)
			# time.sleep(5)
			# motor(0, 0, 1)
			# motor(1, 0, 1)
			# time.sleep(5)
			# motor(0, 1, 0)
			# motor(1, 1, 0)
			# time.sleep(5)
			motor(0, 1, 1)
			time.sleep(0.02)
			motor(1, 1, 1)
			time.sleep(0.02)

		except KeyboardInterrupt:
			GPIO.cleanup()
	return

GPIO.setup(triggerPin, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(echoPin, GPIO.IN)

PWMSetup()
loop()