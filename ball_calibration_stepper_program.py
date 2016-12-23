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
import seaborn as sns
import pickle

# global flags
use_sensor = True
should_plot = True
automatically_test = True
# for more verbose troubleshooting information
__check__ = False
# TODO remove
fixed_side = 'r'

class OpticFlowSensor(serial.Serial):

    def __init__(self, port='/dev/ttyACM0'):
        super(OpticFlowSensor,self).__init__(port=port,baudrate=115200) 
        #super(OpticFlowSensor,self).__init__(port=port,baudrate=1000000) 
        print('Initializing sensor')

        time.sleep(1)
        """

        start = time.time()

        while time.time() < start + 4.0:
            d = self.readData()
            print(d)
            if not len(d) == 0:
                return True

        print('returnin')
        return False
        #raise IOError("not receiving data from sensor")
        """
        print('done')

    def start(self):
        self.write('r')

    def stop(self):
        self.write('s')

    def readData(self):
        dataList = []

        # TODO only do it while bytes inWaiting is a multiple of a line
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

def load_calibration():
    yaw_dims, yaw_coeffs = pickle.load(open('yaw_calibration.p','r'))
    left_y_dim, left_coeff = pickle.load(open('left_calibration.p','r'))
    right_y_dim, right_coeff = pickle.load(open('right_calibration.p','r'))

    coefficients = np.empty((4,1)) * np.nan
    coefficients[yaw_dims] = yaw_coeffs
    coefficients[left_y_dim] = left_coeff
    coefficients[right_y_dim] = right_coeff

    assert not np.any(np.isnan(coefficients)), 'Not all dimensions ' + \
        'covered in calibration.'

    return coefficients

def test_velocity(stepper, sensor, velocity):
    '''
    expecting velocity in revolutions per second.
    '''

    stepper_id = 0
    microsteps_per_rev = 3200
    rotation_time = 5 # sec (per increment)
    acceleration_period = 4 # sec
    deceleration_period = 0 # sec
    # this is the amount of time we will record for
    steady_time = rotation_time - \
        (acceleration_period + deceleration_period)
    # i think this could be avoided if you just figure out
    # how to make acceleration of Phidget non-limiting
    discard_fraction = 0.5

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
    min_so_far = 20
    max_so_far = 0

    if __check__:
        bad_lines = 0

    while time.time() < end:
        # something neater?
        if use_sensor:
            new = sensor.readData()

            sensor_data += [x for x in new if len(x) == 4]

            if __check__:
                for n in new:
                    if len(n) < min_so_far:
                        min_so_far = len(n)
                    if len(n) > max_so_far:
                        max_so_far = len(n)
                    if len(n) != 4:
                        bad_lines += 1

    if __check__:
        print('bad lines in readData: ' + str(bad_lines))
        assert bad_lines < (len(sensor_data) / 2.0), 'bad data'

    if __check__:
        print("smallest number of datapoints from any one read " + \
            str(min_so_far))
        print("largest number of datapoints from any one read " + \
            str(max_so_far))

    # TODO seemed like this might not stop accumulation?
    #if use_sensor:
    #    sensor.stop()

    # I want the sensor to think it is at this position again
    # documentation unclear on effects
    stepper.setCurrentPosition(stepper_id, 0)

    sleep(deceleration_period)

    return sensor_data[int(round(len(sensor_data) * discard_fraction)):]

# TODO docstring
def evaluate_calibration(stepper, sensor, coefficients, dims=None):
    guesses = 5
    
    if dims == None:
        num_dims = 4
        dims = [0, 1, 2, 3]
    else:
        num_dims = dims.size
        if dims.size == 1:
            dims = [dims]

    # same as above
    # corresponds to about 50 cm / sec
    max_revs_per_sec = 16.0

    # from Dylan Chen's answer here:
    # http://stackoverflow.com/questions/6963035/
    # ...pyplot-axes-labels-for-subplots
    fig, ax = plt.subplots(guesses, num_dims, sharex=True, sharey=True)

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

    # to make differences comparable across runs
    np.random.seed(1)
    rand_velocities = np.random.uniform(low=-max_revs_per_sec, \
        high=max_revs_per_sec, size=guesses)

    print('Will try: ' + str(rand_velocities))

    data_dict = dict()
    mean_estimates = np.empty((num_dims, guesses))

    print('num_dims = ' + str(num_dims))
    print('coefficients = ' + str(coefficients))
    print('dims = ' + str(dims))

    diffs = []

    for v in rand_velocities:

        data = test_velocity(stepper, sensor, v)

        '''
        because the bottom of the ball is occluded
        sensors that would otherwise be seeing a net zero
        optic flow will output some nonzero optic flow
        TODO how to calculate velocities such that they
        generalize as best as possible? (to rotation along
        non principal axes)
        '''

        # join all top-level lists within one period
        # of data collection (one rotational velocity)
        #joined = [j for i in trial_data for j in i]

        # for inspection
        all_dim_data = []

        # make one plot for each sensor dimension
        for d in range(num_dims):
            dim_data = []

            for x in data:
                dim_data.append(x[dims[d]])

            dim_data = np.array(dim_data)

            # could try smoothing before scaling

            if should_plot:
                plt.subplot(guesses, num_dims, num_dims*count + d + 1)
                # plotting the error
                # TODO fix: RuntimeWarning: invalid value #
                # encountered in multiply (for a v > 0, < 1)
                plt.plot(np.arange(dim_data.size), \
                    (v - coefficients[d] * dim_data).T, '.', alpha=0.03)


                plt.title('v=' + str(v)[:4] + ', d=' + str(dims[d]))
                # put in a fig note saying d is dimension used to
                # estimate v

                # TODO make all the same and keep for one?
                frame = plt.gca()
                frame.axes.get_xaxis().set_ticks([])
                #frame.axes.get_yaxis().set_ticks([])

            mean_est = coefficients[d] * np.mean(dim_data)
            print('actual velocity : '+ str(v))
            print('mean estimate : ' + \
                str(mean_est))
            print('their difference : ' + str(mean_est - v))
            # TODO median in a sliding window? (could conceivably 
            # actually be used to estimate)
            print('median estimate : ' + \
                str(np.median(coefficients[d] * dim_data)))

            print('')

            diffs.append(abs(mean_est - v))
            mean_estimates[d, count] = mean_est
            all_dim_data.append(dim_data)
            data_dict[v] = all_dim_data

        count = count + 1

        
    if should_plot:
        plt.figure()
        plt.title('Mean linear estimates with single sensor ' +\
            'dimensions')
        plt.xlabel('True velocity')
        plt.ylabel('Estimated velocity')

        indices = np.argsort(rand_velocities)

        # one line for each dimension
        # another line for ground truth
        # each point representing a velocity
        for d in xrange(mean_estimates.shape[0]):
            # too hard to see which points are grouped
            # together without line
            plt.plot(rand_velocities[indices], \
                mean_estimates[d,indices], \
                label='d=' + str(dims[d]))

        plt.legend()
        plt.show()

        print('sum of abs diffs = ' + str(sum(diffs) / ((1.0) * num_dims)))

#Create a stepper object
try:
    stepper = Stepper()
except RuntimeError as e:
    print("Runtime Exception: %s" % e.details)
    print("Exiting....")
    sys.exit(1)

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
    sys.exit(1)

print("Opening phidget object....")

try:
    stepper.openPhidget()
except PhidgetException as e:
    print("Phidget Exception (in opening) %i: %s" % (e.code, e.details))
    print("Exiting....")
    sys.exit(1)

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
        sys.exit(1)

    print("Exiting....")
    sys.exit(1)
else:
    DisplayDeviceInfo()

try:
    mode = raw_input('Select mode:\n[1] - [g]enerate calibration\n' +
                     '[2] - [t]est existing calibration\n' + \
                     '[3] - drive [s]lowly for aligning sensors\n' + 
                     '[4] - [q]uit\n')

    c = mode.lower()
    
    # set some motor parameters largely independent of mode
    stepper_id = 0

    stepper.setCurrentPosition(stepper_id , 0)
    stepper.setAcceleration(stepper_id, 87543)

    # TODO necessary?
    # stepper.setCurrentLimit(stepper_id, 0.5)

    microsteps_per_rev = 3200

    """
    To check whether the ball is in axis. Use in combination with
    Arduino script `vis_optic_flow.ino' and Python script
    `ADNS3080ImageGrabber.py
    """
    if c == '3' or c == 's':
        '''
        # ino build process has not been working
        upload = raw_input('Try uploading vis_optic_flow.ino to' + \
            ' Arduino? [y/n]\n')

        if upload.lower() == 'y':
            subprocess.Popen(['ino','')
        '''

        duration = 120
        velocity = 0.05 # revs / sec

        steps_per_sec = velocity * microsteps_per_rev
        desiredNumMicrosteps = int(round(steps_per_sec * duration))

        stepper.setVelocityLimit(stepper_id, \
            int(steps_per_sec))
        stepper.setTargetPosition(stepper_id, desiredNumMicrosteps)

        stepper.setEngaged(stepper_id, True)
        sleep(duration)

        sys.exit()

    elif c == '4' or c == 'q':
        print('Goodbye!')
        sys.exit()

    elif c == '2' or c == 't':
        """
        Run stepper at random velocities and report on how well
        the existing calibration predicts those values.
        """

        if use_sensor:
            port = '/dev/ttyACM1'
            sensor = OpticFlowSensor(port)
        else:
            sensor = None

        coefficients = load_calibration()

        evaluate_calibration(stepper, sensor, coefficients)
        # TODO or return to menu?
        sys.exit()

    # needs to be the last option. will let me
    # not have to indent the largest block one level further
    elif not c == '1' and not c == 'g':
        print('Invalid option. Goodbye.')
        sys.exit()
    
    if should_plot:
        plt.close('all')
        sns.set_style('dark')

    if fixed_side == None:
        side = raw_input('Where is the calibration stepper, from ' + \
            'your perspective? [l]eft, [r]ight, [t]op\n').lower()
    else:
        side = fixed_side

    if not (side == 'l' or side == 'r' or side == 't'):
        print('Invalid side.')
        sys.exit()
    
    if use_sensor:
        port = '/dev/ttyACM1'
        sensor = OpticFlowSensor(port)
    else:
        sensor = None

    # is this enough to explore sensor behavior near ceiling of 
    # flies velocity?
    # was previously capped by Pavan at 4 revs / sec
    stepper.setVelocityLimit(stepper_id, int(16 * microsteps_per_rev))
    
    stepper.setEngaged(stepper_id, True)
    sleep(1)
    
    # defined purely by above...
    max_revs_per_sec = stepper.getVelocityLimit(0) / microsteps_per_rev

    velocity_increments = 5
    data = []
    velocities = []

    # both directions
    # backward
    for revs_per_sec in np.linspace(max_revs_per_sec, 0, \
        num=velocity_increments, endpoint=False):
        
        new = test_velocity(stepper, sensor, -revs_per_sec)
        data += new
        velocities += [[-revs_per_sec] for x in new]

    # forward
    for revs_per_sec in np.linspace(0, max_revs_per_sec, \
        num=velocity_increments):
        
        new = test_velocity(stepper, sensor, revs_per_sec)
        data += new
        velocities += [[revs_per_sec] for x in new]

    if use_sensor:
        # data = [x for x in data if not np.isclose(x[0], 0.0)]
        rotation_speeds = [x[0] for x in data]
        sensor_dims = 4

        all_dim_data = []
        # will be saved. used to recreate trajectories.
        coefficients = []
        # will be used to determine the dimension to save
        variance = []

        plt.figure()

        for d in range(sensor_dims):
            print('For sensor dimension=' + str(d))

            # M = # data points
            # dim_data should be of shape (M,1)
            # TODO check
            dim_data = []
            for x in data:
                dim_data.append(x[d])

            # TODO check velocities of shape (M,)

            # save the nicely formatted data for use manual inspection later
            all_dim_data.append(dim_data)
            
            plt.subplot(2,2,d+1)
            # TODO wish alpha could be automatically chosen so as to
            # only saturate at the maximum local density of points
            plt.plot(dim_data, '.', alpha=0.01)
            plt.title('Sensor dimension ' + str(d))
            plt.ylabel('Sensor output along dimension '+str(d))
            plt.xlabel('Datapoints from sensor')
            plt.show()
        
            # calculate the coefficient relating the ball velocity, 
            # in revolutions per second, to output along the current
            # sensor dimension
            dim_array = np.array(dim_data).reshape((len(dim_data),1))

            velocities = np.array(velocities)

            X, residuals, rank, singular_values = \
                np.linalg.lstsq(dim_array, velocities)

            # TODO might try RANSAC, or something else less
            # sensitive to outliers than least squares
            # TODO always outputting same singular value?
            # artifact of the numpy algorithm for values near 0?

            # the coefficient relating this sensor dimension
            # to ball velocity in revolutions per second
            # (assumes X is a scalar)
            coefficients.append(X)
            variance.append(np.var(dim_array))

            print('coefficient: ' + str(X))
            print('residual: ' + str(residuals))
            print('variance of sensor data along this dimension: ' + \
                str(variance[-1]))

            # TODO test and report on linearity (r^2?)

            # TODO assert appropriate sensor values are around zero

        plt.show()

        # these will all overwrite existing calibration files
        # values other than l, r, or t, will have quit above already
        if side == 'l':
            # should cause one dimension of ipsilateral sensor
            # to vary the most (dim 1 here)
            left_y_dim = np.argmax(variance)
            print(left_y_dim)
            left_coeff = coefficients[left_y_dim]
            with open('left_sensor_calibration.p', 'w') as f:
                pickle.dump((left_y_dim, left_coeff), f)

        elif side == 'r':
            # should be dim 3 here
            right_y_dim = np.argmax(variance)
            right_coeff = coefficients[right_y_dim]
            print(right_y_dim)
            with open('right_sensor_calibration.p', 'w') as f:
                pickle.dump((right_y_dim, right_coeff), f)
            
        elif side == 't':
            # should vary one dim of each sensor equally (0,2)
            yaw_dims = np.argsort(variance)[0:2]
            print(yaw_dims)
            yaw_coeffs = coefficients[yaw_dims]
            with open('yaw_sensor_calibration.p', 'w') as f:
                pickle.dump((yaw_dims, yaw_coeffs), f)

        if not automatically_test:
            guess = raw_input('Try to guess random rotational ' + \
                'velocities? [y]es / [n]o.\n')
        else:
            print('AUTOMATICALLY TESTING CALIBRATION ALONG THIS DIMENSION')
            print('To disable this, set automatically_test flag to false.')
            guess = 'y'

        # TODO add ability to just test existing calibration

        # should be able to align on different axes before answering
        # the above question, to test a wider array of cases

        # can also switch test balls before starting tests
        # (to check for texture dependence)
        if guess.lower() == 'y':
            if side == 't':
                evaluate_calibration(stepper, sensor, yaw_coeffs, yaw_dims)
            elif side == 'l':
                evaluate_calibration(stepper,sensor,left_coeff, left_y_dim)
            elif side == 'r':
                evaluate_calibration(stepper,sensor,right_coeff,\
                    right_y_dim)

except PhidgetException as e:
    print("Phidget Exception (in main) %i: %s" % (e.code, e.details))

# close serial stuff no matter how we exit
# with(...) syntax probably the best, but more involved
finally:
    stepper.setEngaged(0, False)
    sleep(1)
    stepper.closePhidget()
    
    if use_sensor:
        try:    
            sensor.close()
            print('closed connection to sensor')
        except:
            pass

    print('closed connection to stepper motor')


