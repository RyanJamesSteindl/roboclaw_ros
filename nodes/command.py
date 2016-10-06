#!/usr/bin/env python
#Basic Imports
import math
import rospy
from std_msgs.msg import String
from ctypes import *
import sys
import logging
logging.basicConfig(filename='log.log',level=logging.DEBUG)
#Import methods for sleeping thread
from time import sleep
#Import ROS-MSG Files
from sensor_msgs.msg import Joy
from skupar.msg import Motor_Status
from skupar.msg import Motor_Demand
from geometry_msgs.msg import Twist
#Import Roboclaw motorcontrol Library
import roboclaw
#Init Roboclaw Mode 
address = 0x80 #Mode 7 Packet Serial Mode

global front_demand
global rear_demand

class MCommand():
	
	def __init__(self):
		self.front_status=Motor_Status()
		self.front_demand=Motor_Demand()
		self.rear_status=Motor_Status()
		self.rear_demand=Motor_Demand()

		self.pub_front = rospy.Publisher("front_dmd", Motor_Demand, queue_size=10)
        	self.sub_front=rospy.Subscriber("front_status", Motor_Status, self.callback, self.front_status) 

		self.pub_rear=rospy.Publisher("rear_dmd", Motor_Demand, queue_size=10)
        	self.sub_rear=rospy.Subscriber("rear_status", Motor_Status, self.callback, self.rear_status) 

		self.sub_joy=rospy.Subscriber("joy", Joy, self.joy_callback) 
	
		rospy.init_node('Controller')
	
	def callback(self,core_status,local):
    		#rospy.loginfo(rospy.get_caller_id() + 'I heard %s' %(core_status))
		local.velocity=core_status.velocity
		local.current=core_status.current
		local.error=core_status.error
		local.distance=core_status.distance	
		local.encoderCount=core_status.encoderCount
		local.encoderPosition=core_status.encoderPosition
  
	def joy_callback(self,data):
		#print"Angle: %f Power: %f Axis0: %f Axis1: %f"%(angle, power, data.axes[0], data.axes[1])
		speedscale=1000
		self.front_demand.setVelocityM1 = ((data.axes[1]+data.axes[4])-data.axes[0]-data.axes[3])*speedscale
		self.front_demand.setVelocityM2 = ((data.axes[1]+data.axes[4])+data.axes[0]+data.axes[3])*speedscale
		self.rear_demand.setVelocityM1 = ((data.axes[1]+data.axes[4])+data.axes[0]-data.axes[3])*speedscale
		self.rear_demand.setVelocityM2 = ((data.axes[1]+data.axes[4])-data.axes[0]+data.axes[3])*speedscale
		
		self.pub_front.publish(self.front_demand)
		self.pub_rear.publish(self.rear_demand)	

	def command(self):
	    	while not rospy.is_shutdown():
			r = rospy.Rate(10)
			r.sleep()
    		else:
			print("Done.")
			roboclaw.SpeedM1M2(address,0, 0)
 
	def calcAnglePower(self, data):
		power=math.sqrt(pow(data.axes[0],2) + pow(data.axes[1],2))
		if data.axes[0]<0 and data.axes[1]>0:
			angle=abs(math.atan(data.axes[1]/data.axes[0]))
		elif data.axes[0]==0 and data.axes[1]==1:
			angle=math.pi/2
		elif data.axes[0]>0 and data.axes[1]>0:
			angle=math.atan(data.axes[0]/data.axes[1])+math.pi/2
		elif data.axes[0]==1 and data.axes[1]==0:
			angle=math.pi
		elif data.axes[0]>0 and data.axes[1]<0:
			angle=math.atan(abs(data.axes[1])/data.axes[0])+math.pi
		elif data.axes[0]==0 and data.axes[1]==-1:
			angle=3*math.pi/2		
		elif data.axes[0]<0 and data.axes[1]<0:
			angle=math.atan(data.axes[0]/data.axes[1])+3*math.pi/2
		else:
			angle=0
		return power, angle


if __name__ == '__main__':
    	
	mc = MCommand()
	
	try:
	    	mc.command()
	except rospy.ROSInterruptException:
	    	pass

