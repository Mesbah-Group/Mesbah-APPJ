"""
Testing code for the oscilloscope (PicoScope)

Written/Modified By: Kimberly Chan
(c) 2022 GREMI, University of Orleans
(c) 2022 Mesbah Lab, University of California, Berkeley
"""

import sys
import argparse
import time
from picosdk.ps2000a import ps2000a as ps
import numpy as np
import matplotlib.pyplot as plt
import ctypes

from utils.oscilloscope import Oscilloscope

print("\n--------------------------------")
TEST_STREAMING = True

# Check that the number of arguments is correct
numArg = 2
if len(sys.argv) != numArg:
    print(
        "Function expects "
        + str(numArg - 1)
        + " argument(s). Example: 'oscilloscope_test.py 30' measures oscilloscope for 30 seconds"
    )
    raise

loopTime = int(sys.argv[1])


################################################################################
# USER OPTIONS (you may change these)
################################################################################

# for block mode, you may wish to change the following:
pretrigger_size = 2000  # size of the data buffer before the trigger, default is 2000, in units of samples
posttrigger_size = 8000  # size of the data buffer after the trigger, default is 8000, in units of samples
# for streaming mode, you may wish to change the following:
single_buffer_size = 100  # size of a single buffer, default is 500
n_buffers = 1  # number of buffers to acquire, default is 10
timebase = 127  # 2 corresponds to 4 ns; 127 # 127 corresponds to 1 us

# set the channels to read from oscilloscope
# up to four channels may be set: A, B, C, D
# you should pass a list of dictionaries for the settings of each channel. The
# dictionary for a channel should contain the following keys:
#   "name": the name of the channel, specified either as a single letter
#           ("A", "B", "C", and/or "D") or with the format "CH_A"
#           (i.e., "CH_A", "CH_B", "CH_C", "CH_D"); this is the minimum required
#           for these dictionaries; if only the name is provided, then pre-defined
#           default settings will be used
#   "enable_status": 0 or 1 indicating whether or not to enable this channel;
#           default: 1
#   "coupling_type": AC or DC, specified using the Enums provided in the ps2000a
#           package; default: ps.PS2000A_COUPLING['PS2000A_DC']
#   "range": the range of the signal, spcified using the Enums provided in the
#           ps2000a package; default ps.PS2000A_RANGE['PS2000_2V']
#   "analog_offset": the offset for the analog reading; default: 0.0
channelA = {
    "name": "A",
    "enable_status": 1,
    "coupling_type": ps.PS2000A_COUPLING["PS2000A_DC"],
    "range": ps.PS2000A_RANGE["PS2000A_5V"],
    "analog_offset": 0.0,
}

channelB = {
    "name": "B",
    "enable_status": 1,
    "coupling_type": ps.PS2000A_COUPLING["PS2000A_DC"],
    "range": ps.PS2000A_RANGE["PS2000A_20MV"],
    "analog_offset": 0.0,
}
channelC = {
    "name": "C",
    "enable_status": 1,
    "coupling_type": ps.PS2000A_COUPLING["PS2000A_DC"],
    "range": ps.PS2000A_RANGE["PS2000A_20MV"],
    "analog_offset": 0.0,
}
channelD = {
    "name": "D",
    "enable_status": 1,
    "coupling_type": ps.PS2000A_COUPLING["PS2000A_DC"],
    "range": ps.PS2000A_RANGE["PS2000A_5V"],
    "analog_offset": 0.0,
}

channels = [channelA, channelB, channelC]
# channels = [channelA, channelB, channelC, channelD]
# status = osc.set_channels(channels)
# print(status)

# set the buffers for the data read from the oscilloscope
# each of the channels should be set accordingly, if they were specified previously
# you should pass a list of dictionaries for the settings of each channel. The
# dictionary for the buffer of a channel should contain the following keys:
#   "name": the name of the channel, specified either as a single letter
#           ("A", "B", "C", and/or "D") or with the format "CH_A"
#           (i.e., "CH_A", "CH_B", "CH_C", "CH_D"); this is the minimum required
#           for these dictionaries; if only the name is provided, then pre-defined
#           default settings will be used
#   "segment_index" (or "seg_idx"): default: 0
#   "ratio_mode": ratio mode, specified using the Enums provided in the ps2000a
#           package; default: ps.PS2000A_RATIO_MODE['PS2000A_RATIO_MODE_NONE']
bufferA = {
    "name": "A",
    "segment_index": 0,
    "ratio_mode": ps.PS2000A_RATIO_MODE["PS2000A_RATIO_MODE_NONE"],
}
bufferB = {"name": "B"}
bufferC = {"name": "C"}
bufferD = {"name": "D"}

buffers = [bufferA, bufferB, bufferC]
# buffers = [bufferA, bufferB, bufferC, bufferD]

# a trigger is defined to capture the specific pulse characteristics of the plasma
trigger = {
    "enable_status": 1,
    "source": ps.PS2000A_CHANNEL["PS2000A_CHANNEL_A"],
    "threshold": 1024,  # in ADC counts
    "direction": ps.PS2000A_THRESHOLD_DIRECTION["PS2000A_RISING"],
    "delay": 0,  # in seconds
    "auto_trigger": 200,
}  # in milliseconds

signal = {
    "offsetVoltage": 1000000,  # voltage offset, in microvolts
    "pk2pk": 2000000,  # peak-to-peak voltage of waveform signal (in microvolts)
    "freq": 200,  # frequency of the wavform signal (in Hertz)
    "waveform": ctypes.c_int16(0),  # type of waveform generated
}
# (0) PS2000A_SINE          sine wave
# (1) PS2000A_SQUARE        square wave
# (2) PS2000A_TRIANGLE      triangle wave
# (3) PS2000A_RAMP_UP       rising sawtooth
# (4) PS2000A_RAMP_DOWN     falling sawtooth
# (5) PS2000A_SINC          sin(x)/x
# (6) PS2000A_GAUSSIAN      Gaussian
# (7) PS2000A_HALF_SINE     half (full-wave rectified) sine


################################################################################
# RUN TEST LOOP
# Recommended: Do NOT edit beyond this section
################################################################################
mode = "streaming" if TEST_STREAMING else "block"

# Create an instance of the oscilloscope
osc = Oscilloscope(
    mode=mode, 
    single_buff_size=single_buffer_size, 
    n_buffs=n_buffers,
    pretrigger_size=pretrigger_size, 
    posttrigger_size=posttrigger_size,
)

# Open the oscilloscope
status = osc.open_device()
print(status)
if signal is not None:
    status = osc.set_signal(signal)

# initialize_oscilloscope sets the channels, buffers and trigger (optional)
if TEST_STREAMING:
    status = osc.initialize_device(channels, buffers)
else:
    status = osc.initialize_device(
        channels, buffers, trigger=trigger, timebase=timebase
    )

fig, ax2 = plt.subplots(1, 1, figsize=(10, 6), layout="tight")

plt.ion()
tStart = time.time()
twin_ax = False
while time.time() - tStart <= loopTime:
    s = time.time()
    if TEST_STREAMING:
        t, ch_datas = osc.collect_data_streaming()
    else:
        t, ch_datas = osc.collect_data_block()
    print(f"time to collect data: {time.time()-s}")
    n_channels = len(channels)
    ch_names = ["Ch A", "Ch B", "Ch C", "Ch D"]
    colors = ["tab:blue", "tab:red", "tab:green", "tab:yellow"]
    lines = []
    for i in range(n_channels):
        ch_data = np.asarray(ch_datas[i]["data"])
        if np.any(ch_data > 1e3):
            if not twin_ax:
                ax3 = ax2.twinx()
                twin_ax = True
            l = ax3.plot(t, ch_data / 1e3, lw=2, label=ch_names[i], color=colors[i])
            lines.append(*l)
        else:
            l = ax2.plot(t, ch_data, lw=2, label=ch_names[i], color=colors[i])
            lines.append(*l)
    ax2.set_title(f"Oscilloscope readings from {s} sampling time")
    ax2.set_xlabel("Time (ns)")
    ax2.set_ylabel("Voltage Signal (mV)")
    ax2.set_ylim([-20,20])
    if twin_ax:
        ax3.set_ylabel("Voltage Signal (V)")
    labels = [l.get_label() for l in lines]
    ax2.legend(lines, labels, loc="best")

    plt.draw()
    plt.pause(1)
    ax2.clear()
    if twin_ax:
        ax3.clear()


# stop and close the unit after finished
status = osc.stop_and_close_device()
