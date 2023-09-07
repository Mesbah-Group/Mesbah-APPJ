import serial
import time

import utils.arduino as ard_utils

# Arduino
arduinoAddress = ard_utils.getArduinoAddress(os="ubuntu")
print("Arduino Address: ", arduinoAddress)
arduinoPI = serial.Serial(arduinoAddress, baudrate=38400, timeout=1)

while True:
    out = ard_utils.getMeasArduino(arduinoPI)
    print(list(out).join(',    '))
    time.sleep(1)
