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
def callback(core_status,local_status):
    	rospy.loginfo(rospy.get_caller_id() + 'I heard %s' %(core_status))
	local_status=core_status
   
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
		if x=='w':
			fl_demand.setVelocity=fl_status.velocity+10
			fr_demand.setVelocity=fr_status.velocity+10
			rl_demand.setVelocity=rl_status.velocity+10
			rr_demand.setVelocity=rr_status.velocity+10
		if x=='s':
			fl_demand.setVelocity=fl_status.velocity-10
			fr_demand.setVelocity=fr_status.velocity-10
			rl_demand.setVelocity=rl_status.velocity-10
			rr_demand.setVelocity=rr_status.velocity-10
		if x=='x':
			fl_demand.setVelocity=0
			fr_demand.setVelocity=0
			rl_demand.setVelocity=0
			rr_demand.setVelocity=0
		elif x=='q':
			exit(0)
		#rospy.loginfo(str(mstatus))

		pub_fl.publish(fl_demand)
		pub_fr.publish(fr_demand)
		pub_rl.publish(rl_demand)
		pub_rr.publish(rr_demand)	
	
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

