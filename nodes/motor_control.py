#!/usr/bin/env python

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

from beginner_tutorials.msg import Motor_Status
from beginner_tutorials.msg import Motor_Demand



##########################
def callback(status):
    	rospy.loginfo(rospy.get_caller_id() + 'I heard %s from %s' %(status,name))
    	try:
        	motorControl.setAcceleration(0, 100)
        	motorControl.setVelocity(0, mstatus.getVelocity)
    	except PhidgetException as e:
        	print("Phidget Exception %i: %s" % (e.code, e.details))
   
def motor_status():
	print "Initialising Publisher"
	pub=rospy.Publisher("%s_status" %name, Motor_Status, queue_size=10)
	print "Initialising Subscriber"
        sub=rospy.Subscriber("%s_dmd" %name, Motor_Demand, callback) 

    	speed = 0
    	speed_dmd = 0
    	stop = False
    	samplerate = 0.1
    	e_integral = 0
    	kp = 0.01
    	ki = 0.05
    	PI = 0
    	conv = 170
	maxSpeed = 100
 
	print "Looping on status"
    #samplerate = rospy.Rate(10) # 1hz
    	while not rospy.is_shutdown():
		#print "Motor: " , name, motors[name]['index']
        	try:
	    		encoderVel1 = motorControl.getEncoderPosition(0)
        	except PhidgetException as e:
	    		print("Phidget Exception %i: %s" % (e.code, e.details))
	    		print("Exiting....")
	    		exit(1)
		sleep(samplerate)
        	try:
	    		encoderVel2 = motorControl.getEncoderPosition(0)
        	except PhidgetException as e:
	    		print("Phidget Exception %i: %s" % (e.code, e.details))
	    		print("Exiting....")
	    		exit(1)
		encoderVel = (encoderVel2 - encoderVel1)/samplerate
###################
		speed = float(mstatus.velocity)
		speed = speed*conv
		e = float((speed - encoderVel)/conv)
		e_integral = e_integral + ki*e*samplerate
		PI = kp*e + e_integral
###################
		ctrlSpeed = PI*float(mstatus.velocity)
		if ctrlSpeed > maxSpeed:
			ctrlSpeed = maxSpeed
		if ctrlSpeed < -maxSpeed:
			ctrlSpeed = -maxSpeed
###################
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
		
		r = rospy.Rate(10)
		r.sleep()

    	else:
		print("Done.")
        	try:
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
#####################	
	
    	motorControl = MotorControl()
	mstatus = Motor_Status()
	
	try:
	#logging example, uncomment to generate a log file
	#     motorControl.enableLogging(PhidgetLogLevel.PHIDGET_LOG_VERBOSE, "phidgetlog.log")
	#    motorControl.setOnAttachHandler(motorControlAttached)
	#    motorControl.setOnDetachHandler(motorControlDetached)
	    motorControl.setOnErrorhandler(motorControlError)
	#    motorControl.setOnCurrentChangeHandler(motorControlCurrentChanged)
	#    motorControl.setOnInputChangeHandler(motorControlInputChanged)
	#    motorControl.setOnVelocityChangeHandler(motorControlVelocityChanged)
	except PhidgetException as e:
		print("Phidget Exception %i: %s" % (e.code, e.details))
		print("Exiting....")
		exit(1)

	print("Opening phidget object....")

	#Open the MotorControl Device
	try:
		motorControl.openPhidget(serial)
	except PhidgetException as e:
		print("Phidget Exception on %s %i: %s" % (value, e.code, e.details))
		print("Exiting....")
		exit(1)

#Waiting for the attachment
	rospy.sleep(1)
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
		exit(1)	#Get parameters from ros parameter server for serial number and motor name (topic name)	
	#print("Attaching Motor[%i] = %s\n\n\n\n" %(ind,value))    

	try:
		print "Entering Loop"
	    	motor_status()
	except rospy.ROSInterruptException:
	    	pass

