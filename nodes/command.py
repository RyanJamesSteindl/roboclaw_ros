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
from skupar.msg import Motor_Status
from skupar.msg import Motor_Demand

import roboclaw

address = 0x80

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
	global x
	pub_front=rospy.Publisher("front_dmd", Motor_Demand, queue_size=10)
        sub_front=rospy.Subscriber("front_status", Motor_Status, callback, front_status) 

	pub_rear=rospy.Publisher("rear_dmd", Motor_Demand, queue_size=10)
        sub_rear=rospy.Subscriber("rear_status", Motor_Status, callback, rear_status) 

    	while not rospy.is_shutdown():
		x=raw_input("Distance:")
		x=float(x)
		front_demand.setEncoderValueM1=x
		front_demand.setEncoderValueM2=x
		rear_demand.setEncoderValueM1=x
		rear_demand.setEncoderValueM2=x
		
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
		#rospy.loginfo(str(fr_demand))
		#rospy.loginfo(str(rl_demand))
		#rospy.loginfo(str(rr_demand))

		pub_front.publish(front_demand)
		pub_rear.publish(rear_demand)	
		rospy.loginfo(str(front_status.velocity))
		#rospy.loginfo(str(fl_demand.setVelocity))
		
		r = rospy.Rate(10)
		r.sleep()

    	else:
		print("Done.")
		roboclaw.SpeedM1M2(address,0, 0)

if __name__ == '__main__':
    	
	front_status=Motor_Status()
	front_demand=Motor_Demand()
	rear_status=Motor_Status()
	rear_demand=Motor_Demand()
		
	rospy.init_node('Controller')
	try:
	    	command()
	except rospy.ROSInterruptException:
	    	pass

