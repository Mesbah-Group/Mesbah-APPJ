##################################################################################################################
# THERMAL CAMERA HELPERS
##################################################################################################################

from utils.uvcRadiometry import *


def openThermalCamera():
    ctx = POINTER(uvc_context)()
    dev = POINTER(uvc_device)()
    devh = POINTER(uvc_device_handle)()
    ctrl = uvc_stream_ctrl()

    res = libuvc.uvc_init(byref(ctx), 0)
    if res < 0:
        print("uvc_init error")
        exit(1)

    try:
        res = libuvc.uvc_find_device(ctx, byref(dev), PT_USB_VID, PT_USB_PID, 0)
        if res < 0:
            print("uvc_find_device error")
            exit(1)

        try:
            res = libuvc.uvc_open(dev, byref(devh))
            if res < 0:
                print("uvc_open error")
                exit(1)

            print("device opened!")

      # print_device_info(devh)
      # print_device_formats(devh)

            frame_formats = uvc_get_frame_formats_by_guid(devh, VS_FMT_GUID_Y16)
            if len(frame_formats) == 0:
                print("device does not support Y16")
                exit(1)

            libuvc.uvc_get_stream_ctrl_format_size(devh, byref(ctrl), UVC_FRAME_FORMAT_Y16,
            frame_formats[0].wWidth, frame_formats[0].wHeight, int(1e7 / frame_formats[0].dwDefaultFrameInterval)
            )

            res = libuvc.uvc_start_streaming(devh, byref(ctrl), PTR_PY_FRAME_CALLBACK, None, 0)
            if res < 0:
                print("uvc_start_streaming failed: {0}".format(res))
                exit(1)

      # try:
      #   data = q.get(True, 500)
      #   data = cv2.resize(data[:,:], (640, 480))
      #   minVal, maxVal, minLoc, maxLoc = cv2.minMaxLoc(data)
      #   img = raw_to_8bit(data)
      #   Ts_max = display_temperature(img, maxVal, maxLoc, (0, 0, 255))
      #   Ts_min = display_temperature(img, minVal, minLoc, (255, 0, 0))
            # finally:
            #     pass
      #   libuvc.uvc_stop_streaming(devh)
        finally:
            pass
            # libuvc.uvc_unref_device(dev)
    finally:
        pass
        # libuvc.uvc_exit(ctx)

    return dev, ctx

def getSurfaceTemperature(save_spatial=False, save_image=False):
    data = q.get(True, 500)
    data = cv2.resize(data[:,:], (640, 480))
    minVal, maxVal, minLoc, maxLoc = cv2.minMaxLoc(data)
    img = raw_to_8bit(data)
    Ts_max = display_temperature(img, maxVal, maxLoc, (0, 0, 255))
    Ts_min = display_temperature(img, minVal, minLoc, (255, 0, 0))

    # get offset values of surface temperature (added 2021/03/18)
    # TODO: add spatial measurements to return values as desired
    # 2 pixels away
    n_offset1 = 2
    Ts2 = get_avg_spatial_temp(n_offset1, data, maxLoc)

    # 12 pixels away
    n_offset2 = 12
    Ts3 = get_avg_spatial_temp(n_offset2, data, maxLoc)

    if save_spatial and save_image:
        return Ts_max, (Ts2, Ts3), (data, img)
    elif save_spatial:
        return Ts_max, (Ts2, Ts3)
    elif save_image:
        return Ts_max, (data, img)
    else:
        return Ts_max

def get_avg_spatial_temp(n_pix, data, loc):
    '''
    function to get the average temperature about a certain radius of the
    surface temperature. function gets the values from the four cardinal
    directions and returns the average value of those four values. while the
    function accounts for data out of the image bounds, the best practice is to
    make sure the measured area is near the center of the captured image

    Inputs:
    n_pix        number of pixels offset from the surface temperature measurement
    data        raw image data
    loc            location of the surface temperature measurement

    Outputs:
    avg_temp    average value of the temperature from measurements in the four
                cardinal directions
    '''
    # extract the x and y values from the location
    maxX, maxY = loc
    # east
    if maxX+n_pix >= 640:
        maxValE = ktoc(data[maxY, maxX])
    else:
        maxValE = ktoc(data[maxY, maxX+n_pix])
    # west
    if maxX-n_pix < 0:
        maxValW = ktoc(data[maxY, maxX])
    else:
        maxValW = ktoc(data[maxY, maxX-n_pix])
    # south
    if maxY+n_pix >= 480:
        maxValS = ktoc(data[maxY, maxX])
    else:
        maxValS = ktoc(data[maxY+n_pix, maxX])
    # north
    if maxY-n_pix < 0:
        maxValN = ktoc(data[maxY, maxY])
    else:
        maxValN = ktoc(data[maxY-n_pix, maxX])

    avg_temp = (maxValE+maxValW+maxValN+maxValS)/4
    return avg_temp


def closeThermalCamera(dev, ctx):
    libuvc.uvc_unref_device(dev)
    libuvc.uvc_exit(ctx)