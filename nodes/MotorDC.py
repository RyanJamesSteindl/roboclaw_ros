#!/usr/bin/env python
#Basic imports
from ctypes import *
import sys, os, time
import math 
import time
#ROS imports
import rospy
from roboclaw_ros.msg import Motor_Status
from roboclaw_ros.msg import Motor_Demand
from std_msgs.msg import String
from ctypes import *
#Import Roboclaw API
import roboclaw
import serial

address = 0x80



#Time
global current_time

#Callback for the values from the Command node. These values are written in the local variables from mdemand.
def callback(demand):
    	#rospy.loginfo(rospy.get_caller_id() + 'I heard %s' %(demand))
	#mdemand.setAcceleration = demand.setAcceleration
	mdemand.PWM_M1 = demand.PWM_M1
	mdemand.PWM_M2 = demand.PWM_M2
    
def motor_status():
	currents = roboclaw.ReadCurrents(address)
	if len(currents) > 2:	
		mstatus.currentM1 = currents[1]
		mstatus.currentM2 = currents[2]
	else:
		rospy.loginfo(rospy.get_caller_id() + ': Bad read??? currents len = %d' %len(currents))
	
	mstatus.voltage = roboclaw.ReadMainBatteryVoltage(address)[1]/10.0
	mstatus.header.stamp = rospy.Time.now()
	pub.publish(mstatus)

def drive_motors():
	if mdemand.PWM_M1 >= 0:
		if mdemand.PWM_M1 > 1.0:
			mdemand.PWM_M1 = 1.0
		roboclaw.ForwardM1(address, int(mdemand.PWM_M1*127))
	elif mdemand.PWM_M1 < 0:
		if mdemand.PWM_M1 < -1.0:
			mdemand.PWM_M1 = -1.0
		roboclaw.BackwardM1(address, int(abs(mdemand.PWM_M1)*127))
	
	if mdemand.PWM_M2 >= 0:
		if mdemand.PWM_M2 > 1.0:
			mdemand.PWM_M2 = 1.0
		roboclaw.ForwardM2(address, int(mdemand.PWM_M2*127))
	elif mdemand.PWM_M2 < 0:
		if mdemand.PWM_M2 < -1.0:
			mdemand.PWM_M2 = -1.0
		roboclaw.BackwardM2(address, int(abs(mdemand.PWM_M2)*127))
	

#Main Program Code
if __name__ == '__main__':

	rospy.init_node('motorcontrol', anonymous=True)

	######## Getting the Variables from the Launchfile  
	names = rospy.get_param_names()
	nodename = rospy.get_name()
	name = rospy.get_param('~name')
	USB = rospy.get_param('~USB')
	HZ = rospy.get_param('~Hz')

	rate = rospy.Rate(HZ)
	

	rospy.loginfo(rospy.get_caller_id() + ': opening roboclaw on port: %s' %USB )
	roboclaw.Open("%s" %USB, 38400)

	######## Creating a Motor_Status and Motor_Demand MSG file.
    	mstatus = Motor_Status()
    	mdemand = Motor_Demand()
	
	######## Creating the Publisher and Subscriber
    	pub = rospy.Publisher("%s_status" %name, Motor_Status, queue_size = 10)
    	sub = rospy.Subscriber("%s_dmd" %name,Motor_Demand, callback)

	######## Start up sequence 
	#check Verson
	rospy.loginfo(rospy.get_caller_id() + ': version: %s' %roboclaw.ReadVersion(address)[1] )
	#get config
	rospy.loginfo(rospy.get_caller_id() + ': config: %s' %roboclaw.GetConfig(address)[1] )
    	
	######## Start the control loop
	while not rospy.is_shutdown():    	
		try:
			estatus=0
			drive_motors()
			motor_status()
			rate.sleep()

			
	    	except KeyboardInterrupt:
					rospy.loginfo(rospy.get_caller_id() + ': Ctrl-C detected!!  Exiting...')
					estatus=1
		except Exception as e:
			rospy.loginfo(rospy.get_caller_id() + ': Daymn, something died: %s' %e)
			estatus=2
	
	
	#Stop the Robot after the nodes has been closed
	rospy.loginfo(rospy.get_caller_id() + ': Stopping motors M1 & M2')
	roboclaw.ForwardM1(address, 0)
	roboclaw.ForwardM2(address, 0)

	rospy.loginfo(rospy.get_caller_id() + ': Exiting')
  	exit()

