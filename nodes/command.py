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
def callback(status):
    	rospy.loginfo(rospy.get_caller_id() + 'I heard %s from %s' %(status,name))
		
   
def command():
	pub_fl=rospy.Publisher("front_left_dmd", Motor_Status, queue_size=10)
        sub_fl=rospy.Subscriber("front_left_status", Motor_Demand, callback) 
	pub_fr=rospy.Publisher("front_right_dmd", Motor_Status, queue_size=10)
        sub_fr=rospy.Subscriber("front_right_status", Motor_Demand, callback) 
	pub_rl=rospy.Publisher("rear_left_dmd", Motor_Status, queue_size=10)
        sub_rl=rospy.Subscriber("rear_left_status", Motor_Demand, callback) 
	pub_rr=rospy.Publisher("rear_right_dmd", Motor_Status, queue_size=10)
        sub_rr=rospy.Subscriber("rear_right_status", Motor_Demand, callback) 

    	while not rospy.is_shutdown():
		
		rospy.loginfo(str(mstatus))
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


if __name__ == '__main__':
    	
	motorControl = MotorControl()
	mstatus = Motor_Status()
	try:
	    	command()
	except rospy.ROSInterruptException:
	    	pass

