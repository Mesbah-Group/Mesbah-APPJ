# APPJ Communication

The APPJ is operated via communication to an Arduino. This Arduino accepts commands to change various parameters of the APPJ setup. Additionally, it sends various measured data in a comma-separated line. See the `Firmware` folder for more information and the associated firmware for the Arduino.

Communication to the Arduino can be established by connecting to the device and reading its serial output. This may be achieved directly through the Terminal (Unix)/Command Prompt (Windows)[^1]. The following command(s) may be used:

1. Initialize the connection to the Arduino:

`$stty -f [ARD_ADDR] raw 38400 -hupcl & cat [ARD_ADDR]`

where `[ARD_ADDR]` is the absolute path to the Arduino which includes the device address. An example when connected to a Mac computer may be: `/dev/cu.usbmodem141301` (This changes with device and computer and should be determined when a new connection is made.)

2. Send commands to change operating parameters:

`$echo "[char],[val]" > [ARD_ADDR]`

where `[char]` is a character representing the operating parameter to be changed and `[val]` is the value to which the operating parameter should be changed. A list of the operating parameters may be found in Table 1.

By default, the operating parameters that must be sent to ignite the plasma are the power (w), flow rate (q), and duty cycle (p). Operating parameters are saturated to be within appropriate operating ranges.

**Table 1**: Table of operating parameters for the APPJ.
|Operating Parameter    |Character Representation    |Range [units] (default)  |
|        :----:         |           :----:           |          :----:         |
|Duty Cycle             |p                           |0 – 100 [%] (0)          |
|Primary Gas Flow       |q                           |0 – 10 [slm] (0)         |
|Secondary Gas Flow[^2] |o                           |0 – 20 [sccm] (0)        |
|Frequency              |f                           |10 – 20 [kHz] (20)       |
|Power                  |w                           |1.5 – 5 [W] (0)          |
|X Position[^2]         |x                           |-50 – 50 [mm] (0)        |
|Y Position[^2]         |y                           |-50 – 50 [mm] (0)        |
|Z Position[^2]         |d                           |0 – 20 [mm] (4)          |
|Peak-to-Peak Voltage[^3]|v                          |0 – 10 [kV] (0)          |

[^1]: Windows connections have not been tested.
[^2]: These operating parameters have not been verified for use after the 2020 move to 2nd floor of Tan.
[^3]: The default setup has V14 firmware preloaded to the Arduino (which allows for embedded control of the power via manipulation of the peak-to-peak voltage). Peak-to-peak voltage manipulation is not supported on this version, but is supported with V12 firmware.

Logging data sent to and received from the APPJ may be done via a Python script. This requires the following Python packages to be installed:
* matplotlib[^4]
* numpy[^4]
* scipy[^4]
* seabreeze
* libusb1
* pyserial
* opencv-python[^5]
* opencv-python-headless[^5]
* pyvisa
* python-usbtmc
* pyusb
* crcmod
[^4]: only necessary for data visualization and manipulation
[^5]: choose only one of opencv-python or opencv-python-headless

Additionally, install [`libuvc` from GroupGets](https://github.com/groupgets/libuvc).

Once necessary packages have been installed, connect your device to the setup and preview the `collectDataOL.py` file for an example of how to automate the process of collecting data from the APPJ. In summary, the following components should be included in your script:

1. Create a connection to all devices within setup (Serial connection to Arduino, oscilloscope connection, spectrometer (fiber optic cable OES) connection, thermal camera connection)

2. Create a set of run options (see `RunOpts` class in `APPJFunctions.py`) to decide data collection, data saving, and sampling time of the data collection.

3. Initialize data collection by running the asynchronous task functions once:
(see)

4. Initialize container variables for saving data. **Tip**: It is recommended to initialize the entire array for data collection first rather than appending entries as the code loops.

5. Run experiment:
    
    a. If collecting open loop data, you should initialize various inputs to the APPJ before running the experiment loop
    
    b. If collecting closed loop data, you should initialize all parts of your control scheme (e.g., controller, model, observer/state estimator, etc.) before running the experiment loop (to avoid excess computation time within the sampling loop)
    
    **Tip**: it may be useful to write your own class for experiments to make your code modular. Be sure to pass in all necessary information to this experiment loop (i.e., don’t forget to pass in device information from which to read data)!

6. **SAVE** your data! Remember to put in statements to save all of the desired data you wish to collect. **Tip**: Before running long experiments, make sure this part of your code works!

# Notes/Troubleshooting

## Compatibility with Different Operating Systems (OSs)
Most of the code provided is typically agnostic to different OSs (e.g., Python scripts can be run on any machine, USB devices and serial communication can be achieved on any OS). However, there may be some special modifications to code that is OS-specific (and even OS-version-specific). It is imperative that the user understands how to troubleshoot problems themselves, if using the setup on their own machine. Much of the procedure described above is tested and verified to be successful on macOS Mojave, Catalina, Big Sur[^6], and Monterey[^6].

[^6]: There is a known bug with thermal camera use on these versions.

Additionally, there are known tricks/procedures on Linux distributions:
* By default, Linux distributions lock out USB-connected device permissions for non-superusers. To fix this for each device that is connected to the laptop (Arduino, thermal camera, spectrometer, oscilloscope), you must add udev rules [1, 2, 3].

   - For the spectrometer, after installing seabreeze, run
     
     `$seabreeze_os_setup`
   
   - For the thermal camera, add the following to `/etc/udev/rules.d`
        
        * A file named `99-pt1.rules` with the following line
        
        `SUBSYSTEMS==”usb”, ATTRS{idVendor}==”1e4e”, ATTRS{idProduct}==”0100”, SYMLINK+=”pt1”, GROUP=”usb”, MODE=”666”`
   
   - For [USBTMC devices](https://github.com/python-ivi/python-usbtmc/blob/master/README.md), add the following to `/etc/udev/rules.d`
        
        * A file named `usbtmc.rules` with the following line
        
        `SUBSYSTEMS==”usb”, ACTION==”add”, ATTRS{idVendor}==”0957”, ATTRS{idProduct}==”1755”, GROUP=”usbtmc”, MODE=”0660”`

        **NOTE**: As mentioned in the instructions, you should find the device’s vendor and product info and change them accordingly in the line above. If you have multiple devices, add multiple lines to this file. To find the vendor and product info, connect the device and run `$lsusb`

* The Arduino device name may typically be `/dev/ttyACM0`.

* Only one of the `opencv` packages needs to be installed. `opencv-python` is recommended.

* `maplotlib` requires a GUI backend to work. You can install any; the following command installs one that works
    `$sudo apt-get install python3-tk`

## Other Notes
* The circuitry is FRAGILE; try to touch it as little as possible, and mitigate disturbances to it.
* Sometimes interference causes problems in plasma operation. Foil may potentially be used to insulate the circuits.
* Sometimes the signals are not sent to the system in time; try to increase the sampling time. Sometimes this occurs at the beginning of operation (e.g., the flow rate is not changed at the correct time) and can be fixed by flipping the switch on the mass flow controller.
* Sometimes the thermal camera freezes after too frequent use and/or between infrequent use. This can usually be fixed by resetting it by either unplugging it and plugging it back in or pressing the reset button on the device itself.
* Measurements from the oscilloscope must use a larger sampling time. If these are included in the data collection, make sure to increase the sampling time to roughly 0.3-0.5 seconds per measurement from the oscilloscope.
* Unplugging and plugging back in often fixes many problems.
* The Arduino can be reset by pushing the small red reset button. This will reload the pre-installed firmware and reset all of the operating values to their default values (see Table 1).
