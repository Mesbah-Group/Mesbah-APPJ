##################################################################################################################
# ARDUINO HELPERS
##################################################################################################################

import subprocess
import time
import numpy as np
import crcmod
crc8 = crcmod.predefined.mkCrcFun('crc-8-maxim')


def sendInputsArduino(arduino, appliedPower, flow, dutyCycle, arduinoAddress):
    arduino.reset_input_buffer()
    # Send input values to the microcontroller to actuate them
    subprocess.run('echo "p,{:.2f}" > '.format(dutyCycle) + arduinoAddress, shell=True) #firmware v14
    time.sleep(0.5)
    subprocess.run('echo "w,{:.2f}" > '.format(appliedPower) + arduinoAddress, shell=True) #firmware v14
    time.sleep(0.5)
    subprocess.run('echo "q,{:.2f}" > '.format(flow) + arduinoAddress, shell=True)
    time.sleep(0.5)
    outString = "Input values: Power: %.2f, Flow: %.2f, Duty Cycle: %.2f" %(appliedPower,flow,dutyCycle)
    print(outString)

def sendControlledInputsArduino(arduino, appliedPower, flow, arduinoAddress):
    arduino.reset_input_buffer()
    # Send input values to the microcontroller to actuate them
    subprocess.run('echo "w,{:.2f}" > '.format(appliedPower) + arduinoAddress, shell=True) #firmware v14
    time.sleep(0.05)
    subprocess.run('echo "q,{:.2f}" > '.format(flow) + arduinoAddress, shell=True)
    time.sleep(0.05)
    outString = "Input value(s): Power: %.2f, Flow: %.2f" %(appliedPower,flow)
    print(outString)

def getMeasArduino(dev):
    '''
    function to get embedded measurements from the Arduino (microcontroller)

    Inputs:
    dev     device object for Arduino

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
    '''
    # set default values for data/initialize data values
    Is = 0
    U = [0,0,0,0]    # inputs (applied Voltage, frequency, flow rate)
    x_pos = 0
    y_pos = 0
    dsep = 0
    T_emb = 0
    elec = [0,0]    # electrical measurements (embedded voltage and current)

    # run the data capture
    run = True
    while run:
        try:
            dev.reset_input_buffer()
            dev.readline()
            line = dev.readline().decode('ascii')
            if is_line_valid(line):
                # print(line)
                run = False
                # data read from line indexed as programmed on the Arduino
                V = float(line.split(',')[1])    # p2p Voltage
                f = float(line.split(',')[2])    # frequency
                q = float(line.split(',')[3])    # Helium flow rate
                dsep = float(line.split(',')[4])    # Z position
                Dc = float(line.split(',')[5])    # duty cycle
                Is = float(line.split(',')[6])    # embedded intensity
                V_emb = float(line.split(',')[7])    # embedded voltage
                T_emb = float(line.split(',')[8])    # embedded temperature
                I_emb = float(line.split(',')[9])    # embedded current
                x_pos = float(line.split(',')[10])    # X position
                y_pos = float(line.split(',')[11])    # Y position
                # q2 = float(line.split(',')[12])        # Oxygen flow rate
                Pset = float(line.split(',')[13])    # power setpoint
                P_emb = float(line.split(',')[14])    # embedded power
            else:
                print("CRC8 failed. Invalid line!")
            U = [V,f,q]
            elec = [V_emb, I_emb]
        except Exception as e:
            print(e)
            pass
    print(line)
    return np.array([Is,*U,x_pos,y_pos,dsep,T_emb,P_emb,Pset,Dc,*elec])

def getArduinoAddress(os="macos"):
    '''
    function to get Arduino address. The Arduino address changes each time a new
    connection is made using a either a different computer or USB hub. This
    function works for Unix systems, where devices connected to the computer
    have the path footprint: /dev/...

    UPDATED: 2021/03/18, automatically gets device path (no need for user input)

    Inputs:
    None

    Outputs:
    path of the connected device (Arduino)
    '''
    if os == "macos":
        # command to list devices connected to the computer that can be used as call-out devices
        listDevicesCommand = 'ls /dev/cu.usbmodem*'
        print('Getting devices that under the path: /dev/cu.usbmodem* ...')

    elif os == "ubuntu":
        # command to list devices connected to the computer that can be used as call-out devices
        listDevicesCommand = 'ls /dev/ttyACM*'
        print('Getting devices that under the path: /dev/ttyACM* ...')

    else:
        print('OS not currently supported! Manually input the device path.')

    df = subprocess.check_output(listDevicesCommand, shell=True, text=True)
    devices = []
    for i in df.split('\n'):
        if i:
            devices.append(i)

    if len(devices)>1:
        print('There are multiple devices with this path format.')
        devIdx = int(input('Please input the index of the device that corresponds to the master Arduino (first index = 0):\n'))
    else:
        print('Only one device found! This will be considered the Arduino device.')
        devIdx = 0

    return devices[devIdx]

def is_line_valid(line):
    '''
    Copied from Dogan's code: Verify that the line read from Arduino is complete
    and correct

    Inputs:
    line     line read from Arduino

    Outputs:
    boolean value representing the verification of the line
    '''
    l = line.split(',')
    crc = int(l[-1])
    data = ','.join(l[:-1])
    return crc_check(data,crc)

def crc_check(data,crc):
    '''
    Copied from Dogan's code: Check the CRC value to make sure it's consistent
    with data collected

    Inputs:
    data         line of data collected
    crc         CRC value

    Outputs:
    boolean value representing the verification of the CRC
    '''
    crc_from_data = crc8("{}\x00".format(data).encode('ascii'))
    # print("crc:{} calculated: {} data: {}".format(crc,crc_from_data,data))
    return crc == crc_from_data
