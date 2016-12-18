#!/usr/bin/env python

"""Copyright 2010 Phidgets Inc.
This work is licensed under the Creative Commons Attribution 2.5 Canada License. 
To view a copy of this license, visit http://creativecommons.org/licenses/by/2.5/ca/
"""

#Basic imports
from ctypes import *
import sys
from time import sleep
#Phidget specific imports
from Phidgets.PhidgetException import PhidgetErrorCodes, PhidgetException
from Phidgets.Events.Events import AttachEventArgs, DetachEventArgs, ErrorEventArgs, InputChangeEventArgs, CurrentChangeEventArgs, StepperPositionChangeEventArgs, VelocityChangeEventArgs
from Phidgets.Devices.Stepper import Stepper
from Phidgets.Phidget import PhidgetLogLevel

import serial
# change sleep import above?
import time

import numpy as np
import matplotlib.pyplot as plt

use_sensor = True

class OpticFlowSensor(serial.Serial):

    def __init__(self, port='/dev/ttyACM0'):
        super(OpticFlowSensor,self).__init__(port=port,baudrate=115200) 
        #super(OpticFlowSensor,self).__init__(port=port,baudrate=1000000) 
        print('Initializing sensor')
        time.sleep(4.0)
        print('done')

    def start(self):
        self.write('r')

    def stop(self):
        self.write('s')

    def readData(self):
        dataList = []
        while self.inWaiting() > 0:
            rawData = self.readline()
            rawData = rawData.strip()
            rawData = rawData.split(',')
            try:
                dataList.append([int(x) for x in rawData])
            except:
                pass
        return dataList

def test_velocity(stepper, sensor, velocity):

    microsteps_per_rev = 3200
    rotation_time = 5 # sec (per increment)
    acceleration_period = 2 # sec
    deceleration_period = 1 # sec
    steady_time = rotation_time - \
        (acceleration_period + deceleration_period)

    print 'velocity=' + str(revs_per_sec) + ' revs/sec', 
    if velocity >= 0:
        print 'forward'
    else:
        print 'backward'

    # set a speed
    steps_per_sec = revs_per_sec * microsteps_per_rev
    desiredNumMicrosteps = steps_per_sec * rotation_time

    stepper.setVelocityLimit(stepper_id, steps_per_sec)
    
    # does making it negative work?
    stepper.setTargetPosition(stepper_id, \
        int(np.sign(velocity)) * int(desiredNumMicrosteps))
    
    # wait for it to get up to speed
    sleep(acceleration_period)

    # TODO mind (non-small) synchronization errors
    # acquire optic flow data
    if use_sensor:
        sensor.start()

    end = time.time() + steady_time

    # TODO any way to know arrival time of these? how did ROS add 
    # timestamp?
    sensor_data = []
    while time.time() < end:
        # something neater?
        if use_sensor:
            sensor_data.append(sensor.readData())

    # TODO working? other code didn't seem to use this
    if use_sensor:
        sensor.stop()

    # I want the sensor to think it is at this position again
    # documentation unclear on effects
    stepper.setCurrentPosition(stepper_id, 0)

    sleep(deceleration_period)

    return (revs_per_sec, sensor_data)

#Create a stepper object
try:
    stepper = Stepper()
except RuntimeError as e:
    print("Runtime Exception: %s" % e.details)
    print("Exiting....")
    exit(1)

#Information Display Function
def DisplayDeviceInfo():
    print("|------------|----------------------------------|--------------|------------|")
    print("|- Attached -|-              Type              -|- Serial No. -|-  Version -|")
    print("|------------|----------------------------------|--------------|------------|")
    print("|- %8s -|- %30s -|- %10d -|- %8d -|" % (stepper.isAttached(), stepper.getDeviceName(), stepper.getSerialNum(), stepper.getDeviceVersion()))
    print("|------------|----------------------------------|--------------|------------|")
    print("Number of Motors: %i" % (stepper.getMotorCount()))

#Event Handler Callback Functions
def StepperAttached(e):
    attached = e.device
    print("Stepper %i Attached!" % (attached.getSerialNum()))

def StepperDetached(e):

    detached = e.device
    print("SteppesetTargetPositionr %i Detached!" % (detached.getSerialNum()))

def StepperError(e):
    try:
        source = e.device

        print("Stepper %i: Phidget Error %i: %s" % (source.getSerialNum(), e.eCode, e.description))
    except PhidgetException as e:
        print("Phidget Exception %i: %s" % (e.code, e.details))


def StepperCurrentChanged(e):
    source = e.device
    #print("Stepper %i: Motor %i -- Current Draw: %6f" % (source.getSerialNum(), e.index, e.current))

def StepperInputChanged(e):
    source = e.device
    #print("Stepper %i: Input %i -- State: %s" % (source.getSerialNum(), e.index, e.state))

def StepperPositionChanged(e):
    source = e.device
    #print("Stepper %i: Motor %i -- Position: %f" % (source.getSerialNum(), e.index, e.position))

def StepperVelocityChanged(e):
    source = e.device
    #print("Stepper %i: Motor %i -- Velocity: %f" % (source.getSerialNum(), e.index, e.velocity))

#Main Program Code
try:
	#logging example, uncomment to generate a log file
    #stepper.enableLogging(PhidgetLogLevel.PHIDGET_LOG_VERBOSE, "phidgetlog.log")

    stepper.setOnAttachHandler(StepperAttached)
    stepper.setOnDetachHandler(StepperDetached)
    stepper.setOnErrorhandler(StepperError)
    stepper.setOnCurrentChangeHandler(StepperCurrentChanged)
    stepper.setOnInputChangeHandler(StepperInputChanged)
    stepper.setOnPositionChangeHandler(StepperPositionChanged)
    stepper.setOnVelocityChangeHandler(StepperVelocityChanged)
except PhidgetException as e:
    print("Phidget Exception %i: %s" % (e.code, e.details))
    print("Exiting....")
    exit(1)

print("Opening phidget object....")

try:
    stepper.openPhidget()
except PhidgetException as e:
    print("Phidget Exception %i: %s" % (e.code, e.details))
    print("Exiting....")
    exit(1)

print("Waiting for attach....")

try:
    stepper.waitForAttach(10000)
except PhidgetException as e:
    print("Phidget Exception %i: %s" % (e.code, e.details))
    try:
        stepper.closePhidget()
    except PhidgetException as e:
        print("Phidget Exception %i: %s" % (e.code, e.details))
        print("Exiting....")
        exit(1)
    print("Exiting....")
    exit(1)
else:
    DisplayDeviceInfo()

try:
    should_plot = True

    if use_sensor:
        port = '/dev/ttyACM0'
        sensor = OpticFlowSensor(port)
    else:
        sensor = None
    
    stepper_id = 0

    print("Set the current position as start position...")
    stepper.setCurrentPosition(stepper_id , 0)
    sleep(1)
    
    print("Set the motor as engaged...")
    stepper.setEngaged(stepper_id, True)
    sleep(1)
    
    print("The motor will run until it reaches the set goal position...")
    
    stepper.setCurrentPosition(stepper_id, 0)
    stepper.setAcceleration(stepper_id, 87543)
    # TODO necessary?
    # stepper.setCurrentLimit(stepper_id, 0.5)

    microsteps_per_rev = 3200
    # is this enough to explore sensor behavior near ceiling of 
    # flies velocity?
    # was previously capped by Pavan at 4 revs / sec
    stepper.setVelocityLimit(stepper_id, int(16 * microsteps_per_rev))
    
    # defined purely by above...
    max_revs_per_sec = stepper.getVelocityLimit(0) / microsteps_per_rev

    velocity_increments = 5
    data = []

    # both directions
    # backward
    for revs_per_sec in np.linspace(max_revs_per_sec, 0, \
        num=velocity_increments, endpoint=False):
        
        data.append(test_velocity(stepper, sensor, -revs_per_sec))

    # forward
    for revs_per_sec in np.linspace(0, max_revs_per_sec, \
        num=velocity_increments):
        
        data.append(test_velocity(stepper, sensor, revs_per_sec))

    # TODO save/print coeffs

    if use_sensor:
        # data = [x for x in data if not np.isclose(x[0], 0.0)]
        rotation_speeds = [x[0] for x in data]
        sensor_dims = 4

        all_dim_data = []
        for d in range(sensor_dims):
            print('For sensor dimension=' + str(d))

            # M = # data points
            # dim_data should be of shape (M,1)
            dim_data = []
            # of shape (M,)
            velocities = []
            for v in range(len(data)):
                # group non-empty lists within one velocity
                trial_data = [x for x in data[v][1] if len(x) > 0]

                # join all top-level lists within one period
                # of data collection (one rotational velocity)
                joined = [j for i in trial_data for j in i]

                # select out one dimension for the current regression
                new_dim_data = [x[d] for x in joined]

                # add a number of repeats of the current velocity
                # equal to the number of data points we got here
                # TODO test number of data points is consistent across
                # trials
                velocities = velocities + [[data[v][0]] for x in new_dim_data]
                #dim_data.append(new_dim_data)
                dim_data = dim_data + new_dim_data

            # save the nicely formatted data for use manual inspection later
            all_dim_data.append(dim_data)
            
            plt.figure()
            plt.plot(dim_data)
            plt.title('Sensor dimension ' + str(d))
            plt.show()
        
            # calculate the coefficient relating the ball velocity, 
            # in revolutions per second, to output along the current
            # sensor dimension
            print(np.linalg.lstsq(velocities, \
                dim_data))
            X, residuals, rank, singular_values = np.linalg.lstsq(velocities, \
                dim_data)

            # TODO test and report on linearity (r^2?)

            # TODO assert some sensor values are around zero (yaw ones)

            print(x)
            print(residuals)
    
# TODO only enclose a small section in this try / except block
except PhidgetException as e:
    print("Phidget Exception %i: %s" % (e.code, e.details))

try:
    stepper.setEngaged(0, False)
    sleep(1)
    stepper.closePhidget()
except PhidgetException as e:
    print("Phidget Exception %i: %s" % (e.code, e.details))

print("Done.")
