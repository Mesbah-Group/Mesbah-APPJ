# asynchronous measurement

import asyncio
import time
import cv2
import numpy as np

from utils.thermal_camera import *
from utils.arduino import *

async def async_measure(ard, prevTime, spec, runOpts):
    '''
    function to get measurements from all devices asynchronously to optimize
    time to get measurements

    Inputs:
    ard         Arduino device reference
    spec        Spectrometer device reference
    runOpts     run options; if data should be saved, then measurements will be
                taken, otherwise the task will return None

    Outputs:
    tasks        completed list of tasks containing data measurements; the first
                task obtains temperature measurements, second task obtains
                spectrometer measurements, third task gets oscilloscope
                measurements, and the fourth (final) task gets embedded
                measurements from the Arduino output
    runTime     run time to complete all tasks
    '''
    # create list of tasks to complete asynchronously
    tasks = [asyncio.create_task(async_get_temp(runOpts)),
            asyncio.create_task(async_get_spectra(spec, runOpts)),
            asyncio.create_task(async_get_emb(ard, prevTime, runOpts))]

    startTime = time.time()
    await asyncio.wait(tasks)
    # await asyncio.gather(*tasks)
    endTime = time.time()
    runTime = endTime-startTime
    # print time to complete measurements
    print('...completed data collection tasks after {} seconds'.format(runTime))
    return tasks, runTime

async def async_get_temp(runOpts):
    '''
    asynchronous definition of surface temperature measurement. Assumes the
    camera device has already been initialized. Also can include spatial
    temperature measurements. If spatial temperatures are not desired, then the
    spatial measurements output by this function are -300.

    Inputs:
    runOpts     run options
    **assumes thermal camera device has been successfully opened prior

    Outputs:
    Ts        surface temperature (max temperature from thermal camera) in Celsius
    Ts2        average spatial temperature from 2 pixels away from Ts in Celsius
    Ts3     average spatial temperature from 12 pixels away from Ts in Celsius
    data    raw data matrix of the image captured
    if data collection is specified otherwise, outputs None
    '''
    if runOpts.collectData:
        # run the data capture
        run = True
        while run:
            # image data is processed in a Queue
            data = q.get(True, 500)
            if data is None:
                print("No data read from thermal camera. Check connection.")
                exit(1)
            # data is resized to the appropriate array size
            data = cv2.resize(data[:,:], (640, 480))
            # get min and max values as well as their respective locations
            minVal, maxVal, minLoc, maxLoc = cv2.minMaxLoc(data)
            # convert to Celsius (from Kelvin) with appropriate scaling
            Ts = ktoc(maxVal)

            if runOpts.collectSpatialTemp:
                # get offset values of surface temperature
                # 2 pixels away
                n_offset1 = 2
                Ts2 = get_avg_spatial_temp(n_offset1, data, maxLoc)

                # 12 pixels away
                n_offset2 = 12
                Ts3 = get_avg_spatial_temp(n_offset2, data, maxLoc)
            else:
                Ts2 = -300
                Ts3 = -300

            run = False
        # print('temperature measurement done!')
        return [Ts, Ts2, Ts3, data]
    else:
        return None

async def async_get_spectra(spec, runOpts):
    '''
    asynchronous definition of optical emission spectra data

    Inputs:
    spec         Spectrometer device
    runOpts     run options

    Outputs:
    totalIntensity        total intensity measurement
    intensitySpectrum    intensity spectrum
    wavelengths            wavelengths that correspond to the intensity spectrum
    if data collection is specified otherwise, outputs None
    '''
    if runOpts.collectData:
        intensitySpectrum = spec.intensities()
        meanShift = np.mean(intensitySpectrum[-20:-1])
        intensitySpectrum = intensitySpectrum - meanShift
        totalIntensity = sum(intensitySpectrum[20:])

        if runOpts.collectEntireSpectra:
            wavelengths = spec.wavelengths()
        else:
            wavelengths = None
        # print('spectra recorded!')
        return [totalIntensity, intensitySpectrum, wavelengths, meanShift]
    else:
        return None

async def async_get_emb(dev, prevTime, runOpts):
    '''
    asynchronous definition to get embedded measurements from the Arduino
    (microcontroller)

    Inputs:
    dev         device object for Arduino
    runOpts     run options

    Outputs:
    Outputs:
    Is            embedded surface intensity measurement
    U            inputs (applied peak to peak Voltage, frequency, flow rate)
    x_pos        X position
    y_pos        Y position
    dsep        separation distance from jet tip to substrate (Z position)
    T_emb        embedded temperature measurement
    P_emb        embedded power measurement
    Pset        power setpoint
    Dc            duty cycle
    elec        electrical measurements (embedded voltage and current)
    if data collection is specified otherwise, outputs None
    '''
    if runOpts.collectEmbedded:
        # set default values for data/initialize data values
        Is = 0
        U = [0,0,0]    # inputs (applied Voltage, frequency, flow rate)
        x_pos = 0
        y_pos = 0
        dsep = 0
        T_emb = 0
        elec = [0,0]    # electrical measurements (embedded voltage and current)
        P_emb = 0
        Pset = 0
        Dc = 0

        # run the data capture
        run = True
        while run:
            try:
                # dev.reset_input_buffer()
                # dev.readline()
                line = dev.readline().decode('ascii')
                if is_line_valid(line):
                    # print(line)
                    data = line.split(',')
                    timeStamp = float(data[0])
                    if True:
                    # if (timeStamp-prevTime)/1e3 >= runOpts.tSampling-0.025:
                        run = False
                        # data read from line indexed as programmed on the Arduino
                        V = float(data[1])    # p2p Voltage
                        f = float(data[2])    # frequency
                        q = float(data[3])    # Helium flow rate
                        dsep = float(data[4])    # Z position
                        Dc = float(data[5])    # duty cycle
                        Is = float(data[6])    # embedded intensity
                        V_emb = float(data[7])    # embedded voltage
                        T_emb = float(data[8])    # embedded temperature
                        I_emb = float(data[9])    # embedded current
                        x_pos = float(data[10])    # X position
                        y_pos = float(data[11])    # Y position
                        # q2 = float(data[12])        # Oxygen flow rate
                        Pset = float(data[13])    # power setpoint
                        P_emb = float(data[14])    # embedded power

                        U = [V,f,q]
                        elec = [V_emb, I_emb]
                else:
                    print("CRC8 failed. Invalid line!")
            except Exception as e:
                print(e)
                pass
        print(line)
        # print('embedded measurement done!')
        return np.array([timeStamp, Is,*U,x_pos,y_pos,dsep,T_emb,P_emb,Pset,Dc,*elec])
    else:
        return None
