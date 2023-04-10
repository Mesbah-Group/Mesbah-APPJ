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
import cv2
from datetime import datetime
import asyncio
# pickle import to save class data
try:
    import cPickle as pickle
except ModuleNotFoundError:
    import pickle
import argparse

## import user functions
import utils.APPJPythonFunctions as appj
from utils.experiments import Experiment

sample_num_default = 0 # sample number for treatment
time_treat_default = 30.0 # time to run experiment in seconds
P_treat_default = 2.0 # power setting for the treatment in Watts
q_treat_default = 2.0 # flow setting for the treatment in SLM
dist_treat_default = 5.0 # jet-to-substrate distance in cm

################################################################################
## Set up argument parser
################################################################################
parser = argparse.ArgumentParser(description='Experiment Settings')
parser.add_argument('-n', '--sample_num', type=int, default=sample_num_default,
                    help='The sample number for the test treatments.')
parser.add_argument('-t', '--time_treat', type=float, default=time_treat_default,
                    help='The treatment time desired in seconds.')
parser.add_argument('-p', '--P_treat', type=float, default=P_treat_default,
                    help='The power setting for the treatment in Watts.')
parser.add_argument('-q', '--q_treat', type=float, default=q_treat_default,
                    help='The flow rate setting for the treatment in SLM.')
parser.add_argument('-d', '--dist_treat', type=float, default=dist_treat_default,
                    help='The jet-to-substrate distance in centimeters.')

args = parser.parse_args()
sample_num = args.sample_num
time_treat = args.time_treat
P_treat = args.P_treat
q_treat = args.q_treat
dist_treat = args.dist_treat

print(f"The settings for this treatment are:\n"+
      f"Sample Number:              {sample_num}\n"+
      f"Treatment Time (s):         {time_treat}\n"+
      f"Power (W):                  {P_treat}\n"+
      f"Flow Rate (SLM):            {q_treat}\n"+
      f"Separation Distance (mm):   {dist_treat}\n")

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
runOpts = appj.RunOpts()
runOpts.collectData = True
runOpts.collectEntireSpectra = True
runOpts.collectOscMeas = False
runOpts.collectSpatialTemp = False
runOpts.saveSpectra = True
runOpts.saveOscMeas = False
runOpts.saveSpatialTemp = False
runOpts.saveEntireImage = False
runOpts.tSampling = 1.0

Nsim = int(time_treat/runOpts.tSampling)
ts = runOpts.tSampling

## Set startup values
dutyCycleIn = 100
powerIn = P_treat
flowIn = q_treat

# set save location
directory = os.getcwd()
# os.makedirs(directory+"/ExperimentalData/"+timeStamp, exist_ok=True)
saveDir = directory+"/ExperimentalData/"+timeStamp+f"-Sample{sample_num}/"
print('\nData will be saved in the following directory:')
print(saveDir)

## connect to/open connection to devices in setup
# Arduino
arduinoAddress = appj.getArduinoAddress(os="ubuntu")
print("Arduino Address: ", arduinoAddress)
arduinoPI = serial.Serial(arduinoAddress, baudrate=38400, timeout=1)
s = time.time()
# Oscilloscope
oscilloscope = appj.Oscilloscope()       # Instantiate object from class
instr = oscilloscope.initialize()	# Initialize oscilloscope
# Spectrometer
devices = list_devices()
print(devices)
spec = Spectrometer(devices[0])
spec.integration_time_micros(12000*6)
# Thermal Camera
dev, ctx = appj.openThermalCamera()
print("Devices opened/connected to sucessfully!")

devices = {}
devices['arduinoPI'] = arduinoPI
devices['arduinoAddress'] = arduinoAddress
devices['instr'] = instr
devices['spec'] = spec

# send startup inputs
time.sleep(2)
appj.sendInputsArduino(arduinoPI, powerIn, flowIn, dutyCycleIn, arduinoAddress)
input("Ensure plasma has ignited and press Return to begin.\n")

## Startup asynchronous measurement
if os.name == 'nt':
    ioloop = asyncio.ProactorEventLoop() # for subprocess' pipes on Windows
    asyncio.set_event_loop(ioloop)
else:
    ioloop = asyncio.get_event_loop()
# run once to initialize measurements
prevTime = (time.time()-s)*1e3
tasks, runTime = ioloop.run_until_complete(appj.async_measure(arduinoPI, prevTime, instr, spec, runOpts))
print('measurement devices ready!')
s = time.time()

prevTime = (time.time()-s)*1e3
# get initial measurements
tasks, runTime = ioloop.run_until_complete(appj.async_measure(arduinoPI, prevTime, instr, spec, runOpts))
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
appj.sendInputsArduino(arduinoPI, 0.0, 0.0, dutyCycleIn, arduinoAddress)
arduinoPI.close()
print("Experiments complete!\n"+
    "################################################################################################################\n"+
    "IF FINISHED WITH EXPERIMENTS, PLEASE FOLLOW THE SHUT-OFF PROCEDURE FOR THE APPJ\n"+
    "################################################################################################################\n")
