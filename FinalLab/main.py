import pyrealsense2 as pyreal
import cv2 as cv
import numpy as np
#from maestro import Controller

#tango = Controller()
#motors = 6000
#turns = 6000
#body = 6000

mineArea = False
face = False
color = None

pipeline = pyreal.pipeline()
config = pyreal.config()

qrD = cv.QRCodeDetector()
pipeline_wrapper = pyreal.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(pyreal.camera_info.product_line))

found_rgb = False
for s in device.sensors:
    if s.get_info(pyreal.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)
    
config.enable_stream(pyreal.stream.depth, 640, 480, pyreal.format.z16, 30)

if device_product_line == 'L500':
    config.enable_stream(pyreal.stream.color, 960, 540, pyreal.format.bgr8, 30)
else:
    config.enable_stream(pyreal.stream.color, 640, 480, pyreal.format.bgr8, 30)

counter = 0
# Start streaming
pipeline.start(config)

align_to = pyreal.stream.color
align = pyreal.align(align_to)

#for i in range(20):
    #frames = pipeline.wait_for_frames()
    #depth_frame = frames.get_depth_frame()
    #color_frame = frames.get_color_frame()
frames = pipeline.wait_for_frames()
depth_frame = frames.get_depth_frame()
color_frame = frames.get_color_frame()

color_image = np.asanyarray(color_frame.get_data())
retval, decoded_info, points, straight_qrcode = qrD.detectAndDecodeMulti(color_image)
if retval:
    print("retval: " + str(retval) + ", decode_info: " + str(decoded_info) + ", points: " + str(points) + ", straight_qrcode: " + str(straight_qrcode))

try:
    while True:
        
        frames = pipeline.wait_for_frames()
        #depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        color_image = np.asarray(color_frame.get_data())
        
        #cv.imshow(color_image, "camera feed")
        cv.namedWindow('RealSense', cv.WINDOW_AUTOSIZE)
        cv.imshow('RealSense', color_image)
        
        retval, decoded_info, points, straight_qrcode = qrD.detectAndDecodeMulti(color_image)
        if retval:
            print("retval: " + str(retval) + ", decode_info: " + str(decoded_info) + ", points: " + str(points) + ", straight_qrcode: " + str(straight_qrcode))
        
        cv.waitKey(1)
        if cv.waitKey(1) == 27:
            break
        
finally:
    
    # Stop streaming
    pipeline.stop()
    cv.destroyAllWindows()