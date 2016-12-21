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
import pickle

use_sensor = True
__check__ = False

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
            # TODO just have the arduino output in a binary format
            without_whitespace = rawData.strip()
            without_commas = without_whitespace.split(',')

            integerized = []
            try:
                integerized = [int(x) for x in without_commas]
                dataList.append(integerized)
            except:
                pass
            
            if __check__ and len(integerized) != 4:
                print('bad line: ' + rawData)

        return dataList

def test_velocity(stepper, sensor, velocity):
    '''
    expecting velocity in revolutions per second.
    '''

    microsteps_per_rev = 3200
    rotation_time = 5 # sec (per increment)
    acceleration_period = 2 # sec
    deceleration_period = 1 # sec
    steady_time = rotation_time - \
        (acceleration_period + deceleration_period)

    print 'velocity=' + str(velocity) + ' revs/sec', 
    if velocity >= 0:
        print 'forward'
    else:
        print 'backward'

    # set a speed
    steps_per_sec = velocity * microsteps_per_rev
    desiredNumMicrosteps = steps_per_sec * rotation_time

    # selects only the positive component (magnitude)
    stepper.setVelocityLimit(stepper_id, \
        np.sign(steps_per_sec) * steps_per_sec)
    
    stepper.setTargetPosition(stepper_id, int(desiredNumMicrosteps))
    
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
    min_so_far = 4
    while time.time() < end:
        # something neater?
        if use_sensor:
            new = sensor.readData()
            sensor_data.append(new)

            if __check__:
                for n in new:
                    if len(n) < min_so_far:
                        min_so_far = len(n)

    if __check__:
        print("smallest number of datapoints from any one read " + \
            str(min_so_far))

    # TODO working? other code didn't seem to use this
    # SENSOR SEEMS TO REPORT A SATURATED VALUE RANDOMLY
    # previously seemed as though it might just be after being stopped
    # (not doing this) will just use empty reads in between trials
    #if use_sensor:
    #    sensor.stop()

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
    print("Phidget Exception (in setup) %i: %s" % (e.code, e.details))
    print("Exiting....")
    exit(1)

print("Opening phidget object....")

try:
    stepper.openPhidget()
except PhidgetException as e:
    print("Phidget Exception (in opening) %i: %s" % (e.code, e.details))
    print("Exiting....")
    exit(1)

print("Waiting for attach....")

try:
    stepper.waitForAttach(10000)
except PhidgetException as e:
    print("Phidget Exception (in attaching) %i: %s" % \
        (e.code, e.details))
    # could probably remove this block
    try:
        stepper.closePhidget()
    except PhidgetException as e:
        print("Phidget Exception (in closing) %i: %s" % \
        (e.code, e.details))
        print("Exiting....")
        exit(1)

    print("Exiting....")
    exit(1)
else:
    DisplayDeviceInfo()

try:
    should_plot = True

    if should_plot:
        plt.close('all')

    if use_sensor:
        port = '/dev/ttyACM1'
        sensor = OpticFlowSensor(port)
    else:
        sensor = None
    
    stepper_id = 0

    stepper.setCurrentPosition(stepper_id , 0)
    
    print("Set the motor as engaged...")
    stepper.setEngaged(stepper_id, True)
    sleep(1)
    
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

    if use_sensor:
        # data = [x for x in data if not np.isclose(x[0], 0.0)]
        rotation_speeds = [x[0] for x in data]
        sensor_dims = 4

        all_dim_data = []
        # will be saved. used to recreate trajectories.
        coefficients = []

        plt.figure()

        for d in range(sensor_dims):
            print('For sensor dimension=' + str(d))

            # M = # data points
            # dim_data should be of shape (M,1)
            # TODO check
            dim_data = []

            # of shape (M,)
            # TODO check
            velocities = []

            # each preset velocity stepper runs at
            for v in range(len(data)):
                # group non-empty lists within one velocity
                trial_data = [x for x in data[v][1] if len(x) > 0]

                # join all top-level lists within one period
                # of data collection (one rotational velocity)
                joined = [j for i in trial_data for j in i]

                # select out one dimension for the current regression
                new_dim_data = []
                for x in joined:
                    if len(x) > d:
                        new_dim_data.append(x[d])
                    # sometimes there are malformed lines
                    elif __check__:
                        print(str(x) + " wasn't long enough")

                # add a number of repeats of the current velocity
                # equal to the number of data points we got here
                velocities = velocities + [[data[v][0]] for x in new_dim_data]

                # test number of data points is consistent across
                # trials
                if __check__:
                    print(len(dim_data))

                dim_data = dim_data + new_dim_data

            # save the nicely formatted data for use manual inspection later
            #all_dim_data.append(dim_data)
            
            plt.subplot(2,2,d+1)
            plt.plot(dim_data)
            plt.title('Sensor dimension ' + str(d))
            # TODO WHAT DETERMINES RATE (it is hardcoded in arduino.
            # not sure if it is checked sufficiently though)
            plt.ylabel('Sensor output along dimension '+str(d))
            plt.xlabel('Datapoints from sensor')
            plt.show()
        
            # calculate the coefficient relating the ball velocity, 
            # in revolutions per second, to output along the current
            # sensor dimension
            X, residuals, rank, singular_values = \
                np.linalg.lstsq(velocities, dim_data)
            # TODO might try RANSAC, or something else less
            # sensitive to outliers than least squares
            # TODO always outputting same singular value?
            # artifact of the numpy algorithm for values near 0?

            # the coefficient relating this sensor dimension
            # to ball velocity in revolutions per second
            # (assumes X is a scalar)
            # TODO handle division by zero
            # maybe just run lstqs in the other direction?
            coefficients.append(1 / X)
            print('coefficient: ' + str(1 / X))
            print('residual: ' + str(residuals))

            # TODO test and report on linearity (r^2?)

            # TODO assert appropriate sensor values are around zero

        
        side = raw_input('Where is the calibration stepper, from ' + \
            'your perspective? [l]eft, [r]ight, [t]op, or' + \
            ' do [n]ot save.\n')

        if side.lower() == 'l':
            # should cause one dimension of ipsilateral sensor
            # to vary the most (dim 1 here)
            with open('left_sensor_calibration.p', 'w') as f:
                pickle.dump(coefficients, f)

        elif side.lower() == 'r':
            # should be dim 3 here
            with open('right_sensor_calibration.p', 'w') as f:
                pickle.dump(coefficients, f)
            
        elif side.lower() == 't':
            # should vary one dim of each sensor equally (0,2)
            with open('yaw_sensor_calibration.p', 'w') as f:
                pickle.dump(coefficients, f)

        else:
            print('Not saving calibration data.')

        guess = raw_input('Try to guess random rotational ' + \
            'velocities? [y]es / [n]o.\n')

        # TODO add ability to just test existing calibration

        # should be able to align on different axes before answering
        # the above question, to test a wider array of cases

        # can also switch test balls before starting tests
        # (to check for texture dependence)
        if guess.lower() == 'y':
            guesses = 5
            # same as above
            # corresponds to about 50 cm / sec
            max_revs_per_sec = 16.0

            # from Dylan Chen's answer here:
            # http://stackoverflow.com/questions/6963035/
            # ...pyplot-axes-labels-for-subplots
            fig, ax = plt.subplots(guesses, 4, sharex=True, sharey=True)
            # may not be necessary?
            fig.add_subplot(111, frameon=False)
            plt.tick_params(labelcolor='none', top='off', bottom='off',\
                left='off', right='off')
            # TODO change to dimension / test velocity
            # TODO not working. fix.
            plt.xlabel('Time in 3 second test period')
            plt.ylabel('True velocity (v) - velocity estimated from '+\
                'sensor dimension (d). Units are revs per sec.')
            plt.suptitle('Error in velocity estimated independently ' +\
                'with each dimension')

            count = 0

            rand_velocities = np.random.uniform(low=-max_revs_per_sec, \
                high=max_revs_per_sec, size=guesses)

            print('Will try: ' + str(rand_velocities))

            for v in rand_velocities:

                _, data = test_velocity(stepper, sensor, v)

                '''
                because the bottom of the ball is occluded
                sensors that would otherwise be seeing a net zero
                optic flow will output some nonzero optic flow
                TODO how to calculate velocities such that they
                generalize as best as possible? (to rotation along
                non principal axes)
                '''

                # TODO break this section out in to a function?
                # (it is used above too) just tricky because 
                # it is tightly coupled to velocity enumeration

                # group non-empty lists within one velocity
                trial_data = [x for x in data if len(x) > 0]

                # join all top-level lists within one period
                # of data collection (one rotational velocity)
                joined = [j for i in trial_data for j in i]

                # make one plot for each sensor dimension
                for d in range(4):
                    dim_data = []

                    for x in joined:
                        if len(x) > d:
                            dim_data.append(x[d])

                        # sometimes there are malformed lines
                        elif __check__:
                            print(str(x) + " wasn't long enough")

                    dim_data = np.array(dim_data)

                    # could try smoothing before scaling

                    plt.subplot(guesses, 4, 4*count + d + 1)
                    # plotting the error
                    plt.plot(np.arange(dim_data.size), \
                        v - coefficients[d] * dim_data)

                    plt.title('v=' + str(v)[:4] + ', d=' + str(d))
                    # put in a fig note saying d is dimension used to
                    # estimate v

                    # TODO make all the same and keep for one?
                    frame = plt.gca()
                    frame.axes.get_xaxis().set_ticks([])
                    #frame.axes.get_yaxis().set_ticks([])

                count = count + 1
                
            plt.show()

except PhidgetException as e:
    print("Phidget Exception (in main) %i: %s" % (e.code, e.details))

# close serial stuff no matter how we exit
# with(...) syntax probably the best, but more involved
finally:
    stepper.setEngaged(0, False)
    sleep(1)
    stepper.closePhidget()
    
    if use_sensor:
        sensor.close()
        print('closed connection to sensor')

    print('closed connection to stepper motor')


