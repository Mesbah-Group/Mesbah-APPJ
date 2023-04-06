# Code for running APPJ experiments

*Note: This README assumes that you have already read `../Documentation/APPJ_communication.md` to set up your device to work with the setup.*

This code depends on running the code with all devices connected and within a proper Python environment with appropriate dependencies installed. It is highly recommended to install the dependencies within a virtual environment. This repository comes with support for `conda` environments or self-made environments.

## Via conda
Create a new conda environment and install the required dependencies via
`conda env create -f appj-env.yml`

## Via self-made Python environment
Create a Python environment and activate it. Install the dependencies using
`pip3 install -r appj_requirements.txt`

## Testing Device Connections
Connect your computer to the devices in the APPJ and test the connection by:

1. [SPECTROMETER TEST] Run `spectroscopyLive.py` to test the connection to the spectrometer. You should run this file as any Python file from the command line AND include one additional argument. This argument is a positive number that determines the number of seconds the program will run. If you are connected, the code should run without errors and a figure should pop up displaying the spectra collected by the spectrometer. The window updates roughly every second. The Terminal will also print out the total optical intensity captured (defined as the sum of all intensity values captured by the spectrometer at the given instance). The code also removes the first 20 points.
`python3 spectroscopyLive.py 1000`

2. [IR CAMERA TEST] Change directory (`cd`) into `utils` and run `uvcRadiometry_test.py`. You should run this file as any Python file from the command line without any additional arguments. If you are properly connected, the code will run without errors and a figure should pop up displaying the thermal image being captured by the camera. Furthermore, the maximum and minimum temperatures should be labeled.
`python3 uvcRadiometry_test.py`

3. [ARDUINO TEST] Run `arduino_test.py` to test the connection to the Arduino. You should run this file as any Python file from the command line. If you are connected, the code should run without errors and a line read from the Arduino is printed as output every second. Exit the program with `Ctrl C`
