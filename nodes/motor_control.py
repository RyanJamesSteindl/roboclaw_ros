#!/usr/bin/env python
import time
import rospy
from std_msgs.msg import String
#Basic imports
from ctypes import *
import sys
import logging
logging.basicConfig(filename='log.log',level=logging.DEBUG)
#Phidget specific imports
from Phidgets.PhidgetException import PhidgetErrorCodes, PhidgetException
from Phidgets.Events.Events import AttachEventArgs, DetachEventArgs, ErrorEventArgs, CurrentChangeEventArgs, InputChangeEventArgs, VelocityChangeEventArgs
from Phidgets.Devices.MotorControl import MotorControl
#import methods for sleeping thread
from time import sleep
from Phidgets.Phidget import PhidgetLogLevel

from skupar.msg import Motor_Status
from skupar.msg import Motor_Demand

encoderDmd = 0									#Init Speed Variable
stop = False 									#Init System-Stop
samplerate = 0.01								#Init Samplerate
e_integral = 0									#Init Integral
kp = 100									#Init Prop Gain
ki = 0.1 									#Init Integral Gain
k0 = 1	 									#Overall Gain
output = 0										#Init Controller Variable
conv = 190									#Init Conversion-Variable
maxOutput = 100									#Init Maxoutput
DEADBAND = 2									#Init DeadBand 
encoderVel = 0
encoderPos = 0
encoderPos_old = 0
output_old = 0
output = 0
timesum = 0
time1 = 0
time2 = 0
def PID():

	global e_integral
	global encoderVel
	global output_old	
	global output
	global encoderVel
	global encoderVel_new
	global r
	global encoderPos
	global encoderPos_old
	global timesum
	global time1
	global time2

	encoderPos_old = encoderPos
 	try:
		encoderPos = motorControl.getEncoderPosition(0)		#Get first EncoderPosition Value
	except PhidgetException as e:
		print("Phidget Exception %i: %s" % (e.code, e.details))
		print("Exiting....")
		exit(1)
	encoderVel = (encoderPos - encoderPos_old)/samplerate		#Calculate encoderVelocity
	encoderDmd = mdemand.setVelocity*conv					#Convert Speed Demand
	print "Encoder Demand=%i" %encoderDmd
	e = float((encoderDmd - encoderVel)/conv)				#compute Error
	e = round(e, 0)								#Round Error	
	print "Difference e:%s" %e
	
	output = (kp*e + ki*e_integral)/k0					#Calculate the Velocity Speed
	
#	if abs(e) <= DEADBAND:
#		output = float(encoderDmd)	
	if output > maxOutput:							#Prevent Output from exceeding
		output = maxOutput
	elif output < -maxOutput:
		output = -maxOutput
	else:
		e_integral = e_integral + e*samplerate				#compute Integral Value
	if abs(e)<= DEADBAND:							#Use a DEADBAND for Motorcontrol
		output = mdemand.setVelocity 
		e = 0	
	print"Output: %s"%output
	output = float(output)	
###################
	try:
		if (invert):
			motorControl.setVelocity(0,-output)
	#	elif mdemand.setVelocity == 0:
	#		motorControl.setVelocity(0, 0) 
		else:
			motorControl.setVelocity(0,output)
		#motorControl.setVelocity(0, ctrlSpeed)
		motorControl.setAcceleration(0,100)
	except PhidgetException as e:
		print("Phidget Exception %i: %s" % (e.code, e.details))
		print("Exiting....")
		exit(1)
	




def callback(demand):
    	rospy.loginfo(rospy.get_caller_id() + 'I heard %s' %(demand))
	#speed_dmd=status.setVelocity
	try:
		mdemand.setAcceleration=demand.setAcceleration				#Overwrite Acceleration in new Variable
		mdemand.setVelocity=demand.setVelocity					#Overwrite Velocity in new Variable
	except PhidgetException as e:
		print("Phidget Exception %i: %s" % (e.code, e.details))





def motor_status():
	print "Initialising Publisher"
	pub=rospy.Publisher("%s_status" %name, Motor_Status, queue_size=10)		#Creating Publisher Node
	print "Initialising Subscriber"
        sub=rospy.Subscriber("%s_dmd" %name, Motor_Demand, callback) 			#Creating Subscriber Node
	
	try:
		motorControl.setEncoderPosition(0, 0)
	except PhidgetException as e:
		print("Phidget Exception %i: %s" % (e.code, e.details))
		print("Exiting....")
		exit(1)

	print "Looping on status"
    #samplerate = rospy.Rate(10) # 1hz
    	while not rospy.is_shutdown():

		time1 = time.time()
	 	PID()

		# Add timestamp
		mstatus.header.stamp=rospy.get_rostime()
		print("Encoder Velocity[%s]: %d\n" %(name, encoderVel))
		try:
			mstatus.distance = motorControl.getSensorRawValue(0) 
	        except PhidgetException as e:
	    		print("Phidget Exception %i: %s" % (e.code, e.details))
	    		print("Exiting....")
	    		exit(1)
		try:
			mstatus.velocity = motorControl.getVelocity(0) 
	        except PhidgetException as e:
	    		print("Phidget Exception %i: %s" % (e.code, e.details))
	    		print("Exiting....")
	    		exit(1)
		try:
			mstatus.current = motorControl.getCurrent(0) 
	        except PhidgetException as e:
	    		print("Phidget Exception %i: %s" % (e.code, e.details))
	    		print("Exiting....")
	    		exit(1)
		try:
			mstatus.encoderCount = motorControl.getEncoderCount() 
	        except PhidgetException as e:
	    		print("Phidget Exception %i: %s" % (e.code, e.details))
	    		print("Exiting....")
	    		exit(1)
		try:
			mstatus.encoderPosition = motorControl.getEncoderPosition(0) 
	        except PhidgetException as e:
	    		print("Phidget Exception %i: %s" % (e.code, e.details))
	    		print("Exiting....")
	    		exit(1)
							
		rospy.loginfo(str(mstatus.header.stamp))
		pub.publish(mstatus)
		r.sleep()
		time2 = time.time()							#Calculate the Time before time.sleep(samplerate)
		timesum = time2 - time1	
	
		print"Time Difference:%f"%timesum	
    	else:
		print("Done.")
       	try:
		motorControl.setAcceleration(0, 100)
		motorControl.setVelocity(0, 0)
    		motorControl.closePhidget()
       	except PhidgetException as e:
    		print("Phidget Exception %i: %s" % (e.code, e.details))
    		print("Exiting....")
    		exit(1)





def displayDeviceInfo():
	print("|--------------------------|-------------|----------------------------------|--------------|------------|")
	print("|-        Name            -|-  Attached -|-              Type              -|- Serial No. -|-  Version -|")
	print("|--------------------------|-------------|----------------------------------|--------------|------------|")
	print("|- %25s -|- %9s -|- %30s -|- %10d -|- %8d -|" % (motorControl.isAttached(), motorControl.getDeviceName(), motorControl.getSerialNum(), motorControl.getDeviceVersion()))
	print("|------------|----------------------------------|--------------|------------|")

#Event Handler Callback Functions
def motorControlAttached(e):
	attached = e.device
	print("MotorControl %i Attached!" % (attached.getSerialNum()))

def motorControlDetached(e):
	detached = e.device
	print("MotorControl %i Detached!" % (detached.getSerialNum()))

def motorControlError(e):
	try:
		source = e.device
		print("Motor Control %i: Phidget Error %i: %s" % (source.getSerialNum(), e.eCode, e.description))
	except PhidgetException as e:
		print("Phidget Exception %i: %s" % (e.code, e.details))

def motorControlCurrentChanged(e):
	source = e.device
	mstatus.current = e.current
	print("Motor Control %i: Motor %i Current Draw: %f" % (source.getSerialNum(), e.index, e.current))

def motorControlInputChanged(e):
	source = e.device
	print("Motor Control %i: Input %i State: %s" % (source.getSerialNum(), e.index, e.state))

def motorControlVelocityChanged(e):
	source = e.device
	mstatus.velocity = e.velocity
	print("Motor Control %i: Motor %i Current Velocity: %f" % (source.getSerialNum(), e.index, e.velocity))





if __name__ == '__main__':
#####################
   #Create an motorcontrol object
	rospy.init_node('motorcontrol', anonymous=True)
        print "Initialize Node:"

	names=rospy.get_param_names()
	print "Params  available:" , names

	nodename=rospy.get_name()
	print "Namespace=" , nodename
	name=rospy.get_param('~name')
	print "Name = " , name

	serial=rospy.get_param('~serial_no')
	print "Serial =" , serial
	print "\n\n"

	invert=rospy.get_param('~invert')

#####################	
	
    	motorControl = MotorControl()
	mstatus = Motor_Status()
	mdemand = Motor_Demand()

#####################

	#try:
	#logging example, uncomment to generate a log file
	#    motorControl.enableLogging(PhidgetLogLevel.PHIDGET_LOG_VERBOSE, "phidgetlog.log")
	#    motorControl.setOnAttachHandler(motorControlAttached)
	#    motorControl.setOnDetachHandler(motorControlDetached)
	#     motorControl.setOnErrorhandler(motorControlError)
	#    motorControl.setOnCurrentChangeHandler(motorControlCurrentChanged)
	#    motorControl.setOnInputChangeHandler(motorControlInputChanged)
	#    motorControl.setOnVelocityChangeHandler(motorControlVelocityChanged)
	#except PhidgetException as e:
#		print("Phidget Exception %i: %s" % (e.code, e.details))
#		print("Exiting....")
#		exit(1)

	print("Opening phidget object....")

	#Open the MotorControl Device
	try:
		motorControl.openPhidget(serial)
	except PhidgetException as e:
		print("Phidget Exception on %s %i: %s" % (value, e.code, e.details))
		print("Exiting....")
		exit(1)

#Waiting for the attachment

	r = rospy.Rate(100)
	print("Waiting for attach....")
	rospy.sleep(1)
	try:
		motorControl.waitForAttach(10000)
		print "Attach finished"
	except PhidgetException as e:
		print("Phidget Exception %i: %s" % (e.code, e.details))
		try:
			motorControl.closePhidget()
		except PhidgetException as e:
			print("Phidget Exception %i: %s" % (e.code, e.details))
			print("Exiting....")
			exit(1)
		print("Exiting....")
		exit(1)

	try:
		print "Entering Loop"
	    	motor_status()
	except rospy.ROSInterruptException:
	    	pass

