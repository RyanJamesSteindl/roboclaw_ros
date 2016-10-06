#!/usr/bin/env python
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

# PID Values for the Position and Velocity Control(Position Control is not used in this code) and Acceleleration/Decelleration/Speed Values.These Values are a little bit smaller than the QPPS of the motors for safety reasons(do not break motors).
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

#Values for the Positioncontrol
Deadzone = 15
MaxI = 20
MinPos = 0
MaxPos = 100000

#Callback for the values from the Command node. These values are written in the local variables from mdemand.
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
	if resp[0]:
		# Is Not valid data
		mstatus.encoderCount = resp[2]
		print "Incorrect data ", resp[2]
	mstatus.encoderVelocityM1=roboclaw.ReadSpeedM1(adress)    	
	mstatus.encoderVelocityM2=roboclaw.ReadSpeedM2(adress)    	
	mstatus.encoderPositionM1=roboclaw.ReadEncM1(adress)    	
	mstatus.encoderPositionM2=roboclaw.ReadEncM2(adress)    	

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
	roboclaw.Open("%s" %USB, 38400)
# Setting PID Parameters
	roboclaw.SetM1VelocityPID(address, SKpM1, SKiM1, SKdM1, qppsM1) #Velocity Control Parameter
	roboclaw.SetM2VelocityPID(address, SKpM2, SKiM2, SKdM2, qppsM2) #Velocity Control Parameter
	roboclaw.SetM1PositionPID(address, PKpM1, PKiM1, PKdM1, MaxI, Deadzone, MinPos, MaxPos) #Position Control Parameter
	roboclaw.SetM2PositionPID(address, PKpM2, PKiM2, PKdM2, MaxI, Deadzone, MinPos, MaxPos) #Position Control Parameter
######## Creating a Motor_Status and Motor_Demand MSG file.
    	mstatus = Motor_Status()
    	mdemand = Motor_Demand()
######## Creating the Publisher and Subscriber
    	pub = rospy.Publisher("%s_status" %name, Motor_Status, queue_size = 10)
    	sub = rospy.Subscriber("%s_dmd" %name,Motor_Demand, callback)
	print ("Entering Loop")
#VelocityControl of the motor
	while not rospy.is_shutdown():
		print "requesting VelocityM1: %d VelocityM2: %d Acceleration: %d EncoderValueM1: %d EncoderValueM2: %d" %(mdemand.setVelocityM1, mdemand.setVelocityM2, mdemand.setAcceleration, mdemand.setEncoderValueM1, mdemand.setEncoderValueM2)
		roboclaw.SpeedAccelM1(address, 1200, mdemand.setVelocityM1)
		roboclaw.SpeedAccelM2(address, 1200, mdemand.setVelocityM2)
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
    	roboclaw.SpeedAccelM1(address, 1200, 0)
	roboclaw.SpeedAccelM2(address, 1200, 0)

	print("Closing...")
	print("Done.")
    	exit(estatus)
