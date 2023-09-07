# this file runs the open loop data collection experiments for the APPJ testbed
# in the following paper:
#
#
# Requirements:
# * Python 3
# * several 3rd party packages including CasADi, NumPy, Scikit-Optimize for
# the implemented algorithms and Seabreeze, os, serial, etc. for connection to
# the experimental setup.
#
# Copyright (c) 2021 Mesbah Lab. All Rights Reserved.
# Contributor(s): Kimberly Chan
# Affiliation: University of California, Berkeley
#
# This file is under the MIT License. A copy of this license is included in the
# download of the entire code package (within the root folder of the package).

## import 3rd party packages
import sys
sys.dont_write_bytecode = True
import numpy as np
from seabreeze.spectrometers import Spectrometer, list_devices
import time
import os
import serial
from datetime import datetime
import asyncio
# pickle import to save class data
try:
    import cPickle as pickle
except ModuleNotFoundError:
    import pickle
import argparse
from picosdk.ps2000a import ps2000a as ps

## import user functions
from utils.run_options import RunOpts
from utils.async_measurement import async_measure
import utils.thermal_camera as tc_utils
import utils.arduino as ard_utils
from utils.oscilloscope import Oscilloscope
from utils.experiments import Experiment

SAMPLE_NUMBER = 0 # sample number for treatment
TREATMENT_TIME = 30.0 # time to run experiment in seconds
POWER = 2.0 # power setting for the treatment in Watts
FLOWRATE = 2.0 # flow setting for the treatment in SLM
SEP_DISTANCE = 5.0 # jet-to-substrate distance in cm
SAMPLING_TIME = 1.0 # sampling time for data collection in seconds

################################################################################
## Set up argument parser
################################################################################
parser = argparse.ArgumentParser(description='Experiment Settings')
parser.add_argument('-n', '--sample_num', type=int, default=SAMPLE_NUMBER,
                    help='The sample number for the test treatments.')
parser.add_argument('-t', '--time_treat', type=float, default=TREATMENT_TIME,
                    help='The treatment time desired in seconds.')
parser.add_argument('-p', '--P_treat', type=float, default=POWER,
                    help='The power setting for the treatment in Watts.')
parser.add_argument('-q', '--q_treat', type=float, default=FLOWRATE,
                    help='The flow rate setting for the treatment in SLM.')
parser.add_argument('-d', '--dist_treat', type=float, default=SEP_DISTANCE,
                    help='The jet-to-substrate distance in centimeters.')
parser.add_argument('-ts', '--sampling_time', type=float, default=SAMPLING_TIME,
                    help='The sampling time in seconds.')

args = parser.parse_args()
sample_num = args.sample_num
time_treat = args.time_treat
P_treat = args.P_treat
q_treat = args.q_treat
dist_treat = args.dist_treat
ts = args.sampling_time

print(f"The settings for this treatment are:\n"+
      f"Sample Number:              {sample_num}\n"+
      f"Treatment Time (s):         {time_treat}\n"+
      f"Power (W):                  {P_treat}\n"+
      f"Flow Rate (SLM):            {q_treat}\n"+
      f"Separation Distance (mm):   {dist_treat}\n"+
      f"Sampling Time (s):          {ts}\n")

cfm = input("Confirm these are correct: [Y/n]\n")
if cfm in ['Y', 'y']:
    pass
else:
    quit()

################################################################################
## Startup/prepare APPJ
################################################################################

## collect time stamp
timeStamp = datetime.now().strftime('%Y_%m_%d_%H'+'h%M''m%S'+'s')
print('Timestamp for save files: ', timeStamp)
Nrep = 1

# configure run options
runOpts = RunOpts()
runOpts.collectData = True
runOpts.collectEntireSpectra = True
runOpts.collectOscMeas = False
runOpts.collectSpatialTemp = False
runOpts.saveSpectra = True
runOpts.saveOscMeas = False
runOpts.saveSpatialTemp = False
runOpts.saveEntireImage = False
runOpts.tSampling = ts

Nsim = int(time_treat/runOpts.tSampling)

## Set startup values
dutyCycleIn = 100
powerIn = P_treat
flowIn = q_treat

# set save location
directory = os.getcwd()
saveDir = directory+"/ExperimentalData/"+timeStamp+f"-Sample{sample_num}/"
print('\nData will be saved in the following directory:')
print(saveDir)

## connect to/open connection to devices in setup
# Arduino
arduinoAddress = ard_utils.getArduinoAddress(os="ubuntu")
print("Arduino Address: ", arduinoAddress)
arduinoPI = serial.Serial(arduinoAddress, baudrate=38400, timeout=1)
s = time.time()

# Oscilloscope
## OPTIONAL Configurations for the oscilloscope - in case the settings for the oscilloscope need to be customized
mode = 'block'  # use block mode to capture the data using a trigger; the other option is 'streaming'
# for block mode, you may wish to change the following:
pretrigger_size = 200      # size of the data buffer before the trigger, default is 2000, in units of samples
posttrigger_size = 800     # size of the data buffer after the trigger, default is 8000, in units of samples
# for streaming mode, you may wish to change the following:
single_buffer_size = 500    # size of a single buffer, default is 500
n_buffers = 10              # number of buffers to acquire, default is 10
timebase = 2              # timebase for the measurement resolution, 127 corresponds to 1us, default is 8

# see oscilloscope_test.py for more information on defining the channels
channelA = {"name": "A",
            "enable_status": 1,
            "coupling_type": ps.PS2000A_COUPLING['PS2000A_DC'],
            "range": ps.PS2000A_RANGE['PS2000A_10V'],
            "analog_offset": 0.0,
            }
channelB = {"name": "B",
            "enable_status": 1,
            "coupling_type": ps.PS2000A_COUPLING['PS2000A_DC'],
            "range": ps.PS2000A_RANGE['PS2000A_20V'],
            "analog_offset": 0.0,
            }
channelC = {"name": "C",
            "enable_status": 1,
            "coupling_type": ps.PS2000A_COUPLING['PS2000A_DC'],
            "range": ps.PS2000A_RANGE['PS2000A_20V'],
            "analog_offset": 0.0,
            }
channelD = {"name": "D",
            "enable_status": 0,
            "coupling_type": ps.PS2000A_COUPLING['PS2000A_DC'],
            "range": ps.PS2000A_RANGE['PS2000A_5V'],
            "analog_offset": 0.0,
            }
# put all desired channels into a list (vector with square brackets) named 'channels'
channels = [channelA, channelB, channelC]

# see oscilloscope_test.py for more information on defining the buffers
# a buffer must be defined for every channel that is defined above
bufferA = {"name": "A",
           "segment_index": 0,
           "ratio_mode": ps.PS2000A_RATIO_MODE['PS2000A_RATIO_MODE_NONE'],
           }
bufferB = {"name": "B"}
bufferC = {"name": "C"}
bufferD = {"name": "D"}
# put all buffers into a list (vector with square brackets) named 'buffers'
buffers = [bufferA, bufferB, bufferC]

# see /test/oscilloscope_test.py for more information on defining the trigger (TODO)
# a trigger is defined to capture the specific pulse characteristics of the plasma
trigger = {"enable_status": 1,
           "source": ps.PS2000A_CHANNEL['PS2000A_CHANNEL_A'],
           "threshold": 1024, # in ADC counts
           "direction": ps.PS2000A_THRESHOLD_DIRECTION['PS2000A_RISING'],
           "delay": 0, # in seconds
           "auto_trigger": 200} # in milliseconds

osc = Oscilloscope()
status = osc.open_device()
status = osc.initialize_device(channels, buffers, trigger=trigger, timebase=timebase)


# Spectrometer
devices = list_devices()
print(devices)
spec = Spectrometer(devices[0])
spec.integration_time_micros(12000*6)

# Thermal Camera
dev, ctx = tc_utils.openThermalCamera()
print("Devices opened/connected to sucessfully!")

devices = {}
devices['arduinoPI'] = arduinoPI
devices['arduinoAddress'] = arduinoAddress
devices['osc'] = osc
devices['spec'] = spec

# send startup inputs
time.sleep(2)
ard_utils.sendInputsArduino(arduinoPI, powerIn, flowIn, dutyCycleIn, arduinoAddress)
input("Ensure plasma has ignited and press Return to begin.\n")

## Startup asynchronous measurement
if os.name == 'nt':
    ioloop = asyncio.ProactorEventLoop() # for subprocess' pipes on Windows
    asyncio.set_event_loop(ioloop)
else:
    ioloop = asyncio.get_event_loop()
# run once to initialize measurements
prevTime = (time.time()-s)*1e3
tasks, runTime = ioloop.run_until_complete(async_measure(arduinoPI, prevTime, osc, spec, runOpts))
print('measurement devices ready!')
s = time.time()

prevTime = (time.time()-s)*1e3
# get initial measurements
tasks, runTime = ioloop.run_until_complete(async_measure(arduinoPI, prevTime, osc, spec, runOpts))
if runOpts.collectData:
    thermalCamOut = tasks[0].result()
    Ts0 = thermalCamOut[0]
    specOut = tasks[1].result()
    I0 = specOut[0]
    oscOut = tasks[2].result()
    arduinoOut = tasks[3].result()
    outString = "Measured Outputs: Temperature: %.2f, Intensity: %.2f" % (Ts0, I0)
    print(outString)
else:
    Ts0 = 37
    I0 = 100

s = time.time()

################################################################################
## Begin Experiment:
################################################################################
exp = Experiment(Nsim, saveDir)

for i in range(Nrep):
    s = time.time()

    # create input sequences
    pseq = P_treat*np.ones((Nsim,))
    qseq = q_treat*np.ones((Nsim,))
    print(pseq)
    print(qseq)

    # additional information to save
    opt_dict = {}
    opt_dict['sep_dist'] = dist_treat
    opt_dict['sample_num'] = sample_num

    exp_data = exp.run_open_loop(ioloop,
                                 power_seq=pseq,
                                 flow_seq=qseq,
                                 runOpts=runOpts,
                                 devices=devices,
                                 prevTime=prevTime,
                                 opt_dict=opt_dict)

    arduinoPI.close()

# reconnect Arduino
arduinoPI = serial.Serial(arduinoAddress, baudrate=38400, timeout=1)
devices['arduinoPI'] = arduinoPI

# turn off plasma jet (programmatically)
ard_utils.sendInputsArduino(arduinoPI, 0.0, 0.0, dutyCycleIn, arduinoAddress)
arduinoPI.close()
print("Experiments complete!\n"+
    "################################################################################################################\n"+
    "IF FINISHED WITH EXPERIMENTS, PLEASE FOLLOW THE SHUT-OFF PROCEDURE FOR THE APPJ\n"+
    "################################################################################################################\n")
