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
import logging
logging.basicConfig(filename='log.log',level=logging.DEBUG)
#ROS imports
import rospy
from skupar.msg import Motor_Status
from skupar.msg import Motor_Demand
from std_msgs.msg import String
from ctypes import *
#Phidget specific imports
from Phidgets.PhidgetException import PhidgetErrorCodes, PhidgetException
from Phidgets.Events.Events import AttachEventArgs, DetachEventArgs, ErrorEventArgs, VelocityChangeEventArgs
from Phidgets.Devices.MotorControl import MotorControl
#import methods for sleeping thread
from time import sleep
from Phidgets.Phidget import PhidgetLogLevel

#Initiate GLOBAL variables
command = 0
#PID Values
Kp = 2 
Ki = 0.4
Kd = 0
dt = 24 
errorVelocity = 0
integral = 0
derivative = 0
debugList = []
currentVelocity =0 

def debug():
    for i in debugList:
	print debugList

#Information Display Function
def displayDeviceInfo():
    print("|------------|----------------------------------|--------------|------------|")
    print("|- Attached -|-              Type              -|- Serial No. -|-  Version -|")
    print("|------------|----------------------------------|--------------|------------|")
    print("|- %8s -|- %30s -|- %10d -|- %8d -|" % (motorControl.isAttached(), motorControl.getDeviceName(), motorControl.getSerialNum(), motorControl.getDeviceVersion()))
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

def motorControlPositionChanged(e):
    e.device.setVelocity(0, velocityPID(mdemand.setVelocity, e.time, e.positionChange))

def callback(demand):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s' %(demand))
    try:
        mdemand.setAcceleration = demand.setAcceleration
        mdemand.setVelocity = demand.setVelocity
    except PhidgetException as e:
        print("Phidget Exception %i: %s" % (e.code, e.details))

def motor_status():
   
    try:
        mstatus.distance = motorControl.getSensorRawValue(0)
    except PhidgetException as e:
        print("Phidget Exception %i: %s" % (e.code, e.details))
        print("Exiting....")
        exit(1)
    try:
        mstatus.velocity = currentVelocity 
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
    pub.publish(mstatus)
   # r.sleep
    
def positionPID():
    pass    #TODO: POSITION CONTROL

def velocityPID(targetVelocity, TimeDif, EncoderDif):
    #PID Values
    global Kp
    global Ki
    global Kd
    dt = (TimeDif/3.0)/1000.0

    global errorVelocity
    global integral    
    global derivative
    global debugTuple
    global currentVelocity


    gearRation = 13.7336
    deadband = 2


    CPR = 360.0   #Counts per MOTOR rotation
    maxCountsPerSec = CPR*(2500/60.0)  #Counts a second - 15000
    maxVelocity = math.ceil(maxCountsPerSec/CPR)  #Rev/s 41.667 round to 42

    currentVelocity = (EncoderDif/CPR)/dt

    errorVelocityOld = errorVelocity
    errorVelocity = targetVelocity - currentVelocity
    
    if abs(errorVelocity) < deadband and abs(currentVelocity) < deadband:
        outputVelocity = 0  
    else:
        outputVelocity = (Kp*errorVelocity) + (Ki*integral) + (Kd*derivative)

        if outputVelocity >= maxVelocity:
            outputVelocity = maxVelocity
        elif outputVelocity <= -maxVelocity:
            outputVelocity = -maxVelocity
        else:
            integral += errorVelocity * dt

        derivative = (errorVelocity - errorVelocityOld)/dt

    PWM = int((outputVelocity/float(maxVelocity))*100)
    SetPoint = int((targetVelocity/float(maxVelocity))*100)

    return PWM

    #pygraph =""
    #for i in range(50):
    #    if i == SetPoint/2:
    #        pygraph += '|'
    #    elif i <= PWM/2:
    #        pygraph += '*'  
    #    else:
    #        pygraph += ' '
    #debugList.append(pygraph +'\n')

    #debugList.append("Position Change: " + str(EncoderDif))
    #debugList.append("Current Velocity: " + str(currentVelocity))
    #debugList.append("Error Velocity: " + str(errorVelocity))
    #debugList.append("Output Velocity: " + str(outputVelocity))
    #debugList.append("PWM Velocity: " + PWM)
    #debugList.append("Time Difference: "+ str(dt))
    #debugList.append("\n")

#Main Program Code
if __name__ == '__main__':

    rospy.init_node('motorcontrol', anonymous=True)
########    
    names = rospy.get_param_names()
    nodename = rospy.get_name()
    name = rospy.get_param('~name')
    serial = rospy.get_param('~serial_no')
    invert = rospy.get_param('~invert')
########
    motorControl = MotorControl()
    mstatus = Motor_Status()
    mdemand = Motor_Demand()
########
    pub = rospy.Publisher("%s_status" %name, Motor_Status, queue_size = 10)
    sub = rospy.Subscriber("%s_dmd" %name,Motor_Demand, callback)
 
    try:
	#logging example, uncomment to generate a log file
    #motorControl.enableLogging(PhidgetLogLevel.PHIDGET_LOG_VERBOSE, "phidgetlog.log")

        motorControl.setOnAttachHandler(motorControlAttached)
        motorControl.setOnDetachHandler(motorControlDetached)
        motorControl.setOnErrorhandler(motorControlError)
        motorControl.setOnPositionChangeHandler(motorControlPositionChanged)
    except PhidgetException as e:
        print("Phidget Exception %i: %s" % (e.code, e.details))
        print("Exiting....")
        exit(1)

    print("Opening phidget object....")

    try:
        motorControl.openPhidget(serial)
    except PhidgetException as e:
        print("Phidget Exception %i: %s" % (e.code, e.details))
        print("Exiting....")
        exit(1)

    print("Waiting for attach....")

    try:
        motorControl.waitForAttach(10000)
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

#Control the motor a bit.
    try:
        motorControl.setAcceleration(0, 6200.00)
        while not rospy.is_shutdown():
	    if currentVelocity<=1 and currentVelocity>=-1 :
		motorControl.setVelocity(0, velocityPID(mdemand.setVelocity, dt, 0))
            try:
                motor_status()
	    except:
                print ("Exiting... ")      
                break 
    except PhidgetException as e:
        print("Phidget Exception %i: %s" % (e.code, e.details))

#print("Press Enter to quit....")

#chr = sys.stdin.read(1)

    print("Closing...")

    Target = 0
    sleep(1)

    try:
        motorControl.setVelocity(0, 0)
    except PhidgetException as e:
        print("Phidget Exception %i: %s" % (e.code, e.details))
        print("Exiting....")
        exit(1)

    try:
        motorControl.closePhidget()
    except PhidgetException as e:
        print("Phidget Exception %i: %s" % (e.code, e.details))
        print("Exiting....")
        exit(1)

    print("Done.")
    exit(0)
