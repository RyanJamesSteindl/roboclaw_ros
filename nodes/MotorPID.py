#!/usr/bin/env python

"""Copyright 2010 Phidgets Inc.
This work is licensed under the Creative Commons Attribution 2.5 Canada License. 
To view a copy of this license, visit http://creativecommons.org/licenses/by/2.5/ca/
"""

__author__ = 'Adam Stelmack'
__version__ = '2.1.8'
__date__ = 'May 17 2010'

#Basic imports
from ctypes import *
import sys, os, time
import math 
import time
#ROS imports
import rospy
from skupar.msg import Motor_Status
from skupar.msg import Motor_Demand
from std_msgs.msg import String
from ctypes import *
#Import Roboclaw
import roboclaw
import serial

address = 0x80

# Values
CntM1 = 0

SKpM1 = 700.0
SKiM1 = 10.0
SKdM1 = 0
PKpM1 = 700.0
PKiM1 = 10.0
PKdM1 = 0

AccelM1 = 1800
DeccelM1 = 1800
SpeedM1 = 2100

############
CntM2 = 0

SKpM2 = 700.0
SKiM2 = 10.0
SKdM2 = 0
PKpM2 = 700.0
PKiM2 = 10.0
PKdM2 = 0

AccelM2 = 1800
DeccelM2 = 1800
SpeedM2 = 2100

############
Deadzone = 15
MaxI = 20
MinPos = 0
MaxPos = 100000


def callback(demand):
    	rospy.loginfo(rospy.get_caller_id() + 'I heard %s' %(demand))
        
	#mdemand.setAcceleration = demand.setAcceleration
        mdemand.setVelocityM1 = demand.setVelocityM1
        mdemand.setVelocityM2 = demand.setVelocityM2
        mdemand.setEncoderValueM1 = demand.setEncoderValueM1
        mdemand.setEncoderValueM2 = demand.setEncoderValueM2

def motor_status():
	rm1 = roboclaw.ReadSpeedM1(address)
	if len(rm1) > 2:	
		mstatus.velocity = rm1[2]
        else:
		print ("BAd read??? rm1 len = %d" %len(rm1))
	#mstatus.current = roboclaw.ReadCurrents.cur1(address)
        #resp = roboclaw.ReadEncM1(address)
	if resp[0]:
		# Is Not valid data
		mstatus.encoderCount = resp[2]
		print "Incorrect data ", resp[2]
       	# mstatus.encoderPosition = int(roboclaw.ReadEncM1(address))
    	rospy.sleep(0.05)
	pub.publish(mstatus)



#Main Program Code
if __name__ == '__main__':
	print ("Starting main")
    	rospy.init_node('motorcontrol', anonymous=True)
######## Getting the Variables from the Launchfile   
    	names = rospy.get_param_names()
    	nodename = rospy.get_name()
    	name = rospy.get_param('~name')
    	USB = rospy.get_param('~USB')
	qppsM1 = rospy.get_param('~qppsM1')
	qppsM2 = rospy.get_param('~qppsM2')
########
	print ("opening roboclaw")
	#roboclaw.Open("/dev/ttyACM0", 38400)
	roboclaw.Open("%s" %USB, 38400)
# Setting PID Parameters
	roboclaw.SetM1VelocityPID(address, SKpM1, SKiM1, SKdM1, qppsM1) #Velocity Control Parameter
	roboclaw.SetM2VelocityPID(address, SKpM2, SKiM2, SKdM2, qppsM2) #Velocity Control Parameter
	roboclaw.SetM1PositionPID(address, PKpM1, PKiM1, PKdM1, MaxI, Deadzone, MinPos, MaxPos) #Position Control Parameter
	roboclaw.SetM2PositionPID(address, PKpM2, PKiM2, PKdM2, MaxI, Deadzone, MinPos, MaxPos) #Position Control Parameter
########
    	#motorControl = MotorControl()
    	mstatus = Motor_Status()
    	mdemand = Motor_Demand()
	#mdemand. setVelocity = 0
########
    	pub = rospy.Publisher("%s_status" %name, Motor_Status, queue_size = 10)
    	sub = rospy.Subscriber("%s_dmd" %name,Motor_Demand, callback)
	#sub = rospy.Subscriber("joystick", joy_node, callback)
	#mdemand.setEncoderValue = int(mdemand.setEncoderValue)
 
	print ("Entering Loop")
#Control the motor a bit.
	while not rospy.is_shutdown():
		print "requesting VelocityM1: %d VelocityM2: %d Acceleration: %d EncoderValueM1: %d EncoderValueM2: %d" %(mdemand.setVelocityM1, mdemand.setVelocityM2, mdemand.setAcceleration, mdemand.setEncoderValueM1, mdemand.setEncoderValueM2)
		
            	#roboclaw.SpeedAccelDeccelPositionM1M2(address, AccelM1, SpeedM1, DeccelM1, mdemand.setEncoderValueM1, AccelM2, SpeedM2, DeccelM2,mdemand.setEncoderValueM2, 0)
		roboclaw.SpeedAccelM1(address, 1200, mdemand.setVelocityM1)
		roboclaw.SpeedAccelM2(address, 1200, mdemand.setVelocityM2)
		#roboclaw.SpeedAccelM1M2(address, 1200, mdemand.setVelocityM1, mdemand.setVelocityM2)
	    	try:
			estatus=0
                	motor_status()
			print "EncoderValue M1: %i" %roboclaw.ReadSpeedM1(address)[2]
	    	except KeyboardInterrupt:
                	print ("Ctrl-C detected!!  Exiting... ")      
                	estatus=1
		except Exception as e:
			print ("Oh darn, something died: %s " % e)
			estatus=2
#print("Press Enter to quit....")

#chr = sys.stdin.read(1)

    	print("Closing...")

	#Slow Motor Down to Zero
        #roboclaw.SpeedAccelDeccelPositionM1M2(address, AccelM1, SpeedM1, DeccelM1, 0, AccelM2, SpeedM2, DeccelM2, 0, 0)
	#roboclaw.Close()
    	
	print("Done.")
    	exit(estatus)
