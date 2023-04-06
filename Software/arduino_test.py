import serial
import time

import utils.APPJPythonFunctions as appj





# Arduino
arduinoAddress = appj.getArduinoAddress(os="ubuntu")
print("Arduino Address: ", arduinoAddress)
arduinoPI = serial.Serial(arduinoAddress, baudrate=38400, timeout=1)

while True:
    appj.getMeasArduino(arduinoPI)
    time.sleep(1)
