#!/usr/bin/env python

import rospy
from std_msgs.msg import String
#Basic imports
from ctypes import *
import sys
import logging
logging.basicConfig(filename='log.log',level=logging.DEBUG)
#Curses import and initialize
#import curses
#stdscr = curses.initscr()
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
def callback(core_status,local):
    	#rospy.loginfo(rospy.get_caller_id() + 'I heard %s' %(core_status))
	local.velocity=core_status.velocity
	local.current=core_status.current
	local.error=core_status.error
	local.distance=core_status.distance	
	local.encoderCount=core_status.encoderCount
	local.encoderPosition=core_status.encoderPosition
   
def command():
	
	pub_fl=rospy.Publisher("front_left_dmd", Motor_Demand, queue_size=10)
        sub_fl=rospy.Subscriber("front_left_status", Motor_Status, callback, fl_status) 
	pub_fr=rospy.Publisher("front_right_dmd", Motor_Demand, queue_size=10)
        sub_fl=rospy.Subscriber("front_right_status", Motor_Status, callback, fr_status) 
	pub_rl=rospy.Publisher("rear_left_dmd", Motor_Demand, queue_size=10)
        sub_rl=rospy.Subscriber("rear_left_status", Motor_Status, callback, rl_status) 
	pub_rr=rospy.Publisher("rear_right_dmd", Motor_Demand, queue_size=10)
        sub_rr=rospy.Subscriber("rear_right_status", Motor_Status, callback, rr_status) 

    	while not rospy.is_shutdown():
		x=raw_input("Speed:")

		x=float(x)
		fl_demand.setVelocity=x
		fr_demand.setVelocity=x
		rl_demand.setVelocity=x
		rr_demand.setVelocity=x

		#if x=='w':
		#	fl_demand.setVelocity=fl_status.velocity+40
		#	fr_demand.setVelocity=fr_status.velocity+40
		#	rl_demand.setVelocity=rl_status.velocity+40
		#	rr_demand.setVelocity=rr_status.velocity+40
		#if x=='s':
		#	fl_demand.setVelocity=fl_status.velocity-40
		#	fr_demand.setVelocity=fr_status.velocity-40
		#	rl_demand.setVelocity=rl_status.velocity-40
		#	rr_demand.setVelocity=rr_status.velocity-40
		#if x=='x':
		#	fl_demand.setVelocity=0
		#	fr_demand.setVelocity=0
		#	rl_demand.setVelocity=0
		#	rr_demand.setVelocity=0
		#elif x=='q':
	#		exit(0)
		rospy.loginfo(str(fl_status.velocity))
		rospy.loginfo(str(fl_demand.setVelocity))
		#rospy.loginfo(str(fr_demand))
		#rospy.loginfo(str(rl_demand))
		#rospy.loginfo(str(rr_demand))

		pub_fl.publish(fl_demand)
		pub_fr.publish(fr_demand)
		pub_rl.publish(rl_demand)
		pub_rr.publish(rr_demand)	
	
		r = rospy.Rate(10)
		r.sleep()

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


if __name__ == '__main__':
    	
	motorControl = MotorControl()
	fl_status=Motor_Status()
	fr_status=Motor_Status()
	rr_status=Motor_Status()
	rl_status=Motor_Status()
	fl_demand=Motor_Demand()
	fr_demand=Motor_Demand()
	rl_demand=Motor_Demand()
	rr_demand=Motor_Demand()
	
	rospy.init_node('Controller')
	try:
	    	command()
	except rospy.ROSInterruptException:
	    	pass

