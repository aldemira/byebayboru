#!/usr/bin/python

import thread
import time
import RPi.GPIO as GPIO
import random

# Setup
random.seed()
GPIO.setmode(GPIO.BOARD)
GPIO.setup(11, GPIO.OUT) # Led
my_pwm = GPIO.PWM(11,100)
my_pwm.start(0)

def set(property, value):
	try:
		f = open("/sys/class/rpi-pwm/pwm0/" + property, 'w')
		f.write(value)
		f.close()	
	except:
		print("Error writing to: " + property + " value: " + value)
 
 
def setServo(angle):
	set("servo", str(angle))
	
		

def talk_to_me():
	myangle = random.randint(0, 180)
	setServo(myangle)
	time.sleep(0.020)
	return

def blink_me_heart():
	for i in range(1,10):
		my_pwm.ChangeDutyCycle(10*i)
	time.sleep(1)
	for i in range(10,1):
		my_pwm.ChangeDutyCycle(10*i)

	my_pwm.ChangeDutyCycle(0)
	time.sleep(1)
	return

set("delayed", "0")
set("mode", "servo")
set("servo_max", "180")
set("active", "1")
 
try:
	thread.start_new_thread(talk_to_me)
	thread.start_new_thread(blink_me_heart)

except:
	print "Thread error"

while 1:
	pass
