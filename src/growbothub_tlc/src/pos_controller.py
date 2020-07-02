#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO
from growbot_msg.msg import Wheel_moving
from growbot_msg.msg import Wheel_target
import constants as cst 

GPIO.setmode(GPIO.BOARD)

# GLOBAL VARIABLES
pos = 0
pwm = None
target = 0
posSaveFileName = "/home/pi/ros_catkin_ws/src/growbot_rpi/sensorLog/posWhenShutdown.txt"
try:
	f = open(posSaveFileName, "r")
	pos = int(f.read())
	rospy.loginfo("Wheel starts at position : " + str(pos))
	f.close()
except Exception as e:
	rospy.logwarn("Unable to find : " + posSaveFileName)
	pos = 0

distanceTraveled = 0

pub_done = 0
pub_target = 0


def saturatedSpeed(s):
	if abs(s) > cst._MAX_SPEED :
		return cst._MAX_SPEED
	elif abs(s) < cst._MIN_SPEED :
		return cst._MIN_SPEED
	else :
		return s

def cb_counter(channel):
	global pos
	global distanceTraveled

	if GPIO.input(cst._PIN_ENC_B) :
		pos = pos - 1
	else :
		pos = pos + 1

	if pos % 10 == 0 :
		pass
		#print("Current pos : " + str(pos))

	distanceTraveled = distanceTraveled + 1
	goTo()


def cb_alm(channel):
	rospy.logerr("Driver sent ALARM signal !")
	#GPIO.cleanup()


def cb_but_awo(channel):
	rospy.sleep(0.1)
	if GPIO.input(channel) == GPIO.HIGH :
		rospy.loginfo("Winding switched off")
		GPIO.output(cst._PIN_BUTAWO, GPIO.HIGH)
	else :
		rospy.loginfo("Winding switched on")
		rospy.logwarn("Current position set as 0, target set to 0")
		GPIO.output(cst._PIN_BUTAWO, GPIO.LOW)
		global pos
		global distanceTraveled
		pos = 0
		distanceTraveled = 0
		msg = Wheel_target()
		msg.target = 0
		rospy.loginfo("Msg published")
		pub_target.publish(msg)


def pinSetup():
	GPIO.setup(cst._PIN_TIM, GPIO.OUT)
	GPIO.setup(cst._PIN_TIM_G, GPIO.OUT)
	GPIO.setup(cst._PIN_CS, GPIO.OUT)
	GPIO.setup(cst._PIN_CS_G, GPIO.OUT)
	GPIO.setup(cst._PIN_AWO_G, GPIO.OUT)
	GPIO.setup(cst._PIN_BUTAWO, GPIO.OUT)
	GPIO.setup(cst._PIN_ALM_G, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
	GPIO.setup(cst._PIN_BUT_IN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
	GPIO.setup(cst._PIN_BUT_LED, GPIO.OUT)

	GPIO.output(cst._PIN_TIM, GPIO.LOW)
	GPIO.output(cst._PIN_TIM_G, GPIO.LOW)
	GPIO.output(cst._PIN_CS, GPIO.LOW)
	GPIO.output(cst._PIN_CS_G, GPIO.LOW)
	GPIO.output(cst._PIN_AWO_G, GPIO.LOW)
	GPIO.output(cst._PIN_BUTAWO, GPIO.LOW)
	GPIO.output(cst._PIN_BUT_LED, GPIO.HIGH)

	GPIO.setup(cst._PIN_ENC_A, GPIO.IN)
	GPIO.setup(cst._PIN_ENC_B, GPIO.IN)
	GPIO.setup(cst._PIN_ENC_Z, GPIO.IN)

	GPIO.setup(cst._PIN_DIR, GPIO.OUT)
	GPIO.setup(cst._PIN_PWM, GPIO.OUT)

	GPIO.add_event_detect(cst._PIN_ENC_A, GPIO.RISING, callback=cb_counter)
	GPIO.add_event_detect(cst._PIN_ALM_G, GPIO.FALLING, callback=cb_alm)
	#GPIO.add_event_detect(cst._PIN_BUT_IN, GPIO.FALLING, callback=cb_but_awo_falling, bouncetime=200)
	GPIO.add_event_detect(cst._PIN_BUT_IN, GPIO.BOTH, callback=cb_but_awo, bouncetime=200)
	#GPIO.add_event_detect(cst._PIN_AWO_G, GPIO.FALLING 	, callback=cb_awo)  
	global pwm
	pwm = GPIO.PWM(cst._PIN_PWM, 5)


def goTo(init=False):
	global distanceTraveled
	if init == True :
		distanceTraveled = 0
		msg = Wheel_moving()
		msg.isMoving = True
		pub_done.publish(msg)
		rospy.loginfo("New target recieved : " + str(target))

	error = target - pos
	pwm.start(cst._DUTY_CYCLE)

	if error > 0 :
		GPIO.output(cst._PIN_DIR, True)
	else :
		GPIO.output(cst._PIN_DIR, False)

	if not error == 0 :
		#print("error : %d", error)
		pwm.ChangeFrequency(min([saturatedSpeed(abs(error)), cst._ACCELERATION_FACTOR*saturatedSpeed(distanceTraveled)]))
		error = target - pos
	else :
		pwm.stop()
		rospy.loginfo("Target reached, current pos : " + str(pos))
		msg = Wheel_moving()
		msg.isMoving = False
		rospy.sleep(0.1)
		pub_done.publish(msg)

def cb_target(data):
	global target
	if data.target < 0 or data.target >= cst._P_PER_ROTATION :
		rospy.logwarn("Out of range target recieved, command ignored...")
	else :
		target = data.target
		goTo(init=True)

def initPublisher():
    global pub_done
    global pub_target
    pub_done = rospy.Publisher('/wheel/done', Wheel_moving, queue_size=10)
    pub_target = rospy.Publisher('/wheel/target', Wheel_target, queue_size=10)

    
def initSubscriber():
	rospy.Subscriber("/wheel/target", Wheel_target, cb_target)

def initServices():
    #rospy.Service('/wheel/goTo', ImPro_doImPro, srvHdl_goTo)
    pass
    

def main():
	global target
	pinSetup()
	initSubscriber()
	initPublisher()
	#target = 1000
	#goTo(init=True)
	rospy.loginfo("wheel_controller : Running...")
	rospy.spin()


if __name__ == '__main__':
	try:
		rospy.init_node('wheel_controller', anonymous=True)
		main()
		f = open(posSaveFileName, "w")
		f.write(str(target))
		f.close()
		GPIO.cleanup()
	except rospy.ROSInterruptException:
		pass
