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
oriented = False

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
        depth_image = np.asanyarray(depth_frame.get_data())
        
        depth_colormap = cv.applyColorMap(cv.convertScaleAbs(depth_image, alpha=0.03), cv.COLORMAP_JET)

        depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = color_image.shape

        if depth_colormap_dim != color_colormap_dim:
            resized_color_image = cv.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv.INTER_AREA)
            images = np.hstack((resized_color_image, depth_colormap))
        else:
            images = np.hstack((color_image, depth_colormap))
        hsv = cv.cvtColor(color_image, cv.COLOR_BGR2HSV)
        
        cv.namedWindow('RobotVision', cv.WINDOW_AUTOSIZE)
        cv.imshow('RobotVision', color_image) 
        cv.waitKey(1)
        
        #cv.imshow(color_image, "camera feed")
        cv.namedWindow('RealSense', cv.WINDOW_AUTOSIZE)
        cv.imshow('RealSense', color_image)
        
        normalize = np.zeros((680, 480))
        normalize = cv.normalize(color_image, normalize, 0,255, cv.NORM_MINMAX)
        gray = cv.cvtColor(normalize,cv.COLOR_BGR2GRAY)
        blur = cv.GaussianBlur(gray,(5,5), 0)
        edge = cv.Canny(blur,100,200)
        (t, threshold) = cv.threshold(edge, 0,255,cv.THRESH_BINARY)
        Moments = cv.moments(threshold)
        
        if Moments["m00"] != 0:
            cX = int(Moments["m10"] / Moments["m00"])
            cY = int(Moments["m01"] / Moments["m00"])
        else:
            cX, cY = 0,0
            
        cv.circle(threshold, (cX, cY), 5, (255, 255, 255), -1)
            
        if oriented is False:
            retval, decoded_info, points, straight_qrcode = qrD.detectAndDecodeMulti(color_image)
            if decoded_info == '1':
                #turn left until see qr Code
                #motors = 6400
                #turns = 5600
                
                #tango.setTarget(MOTORS, motors)
                #tango.setTarget(TURN,turns)
                #print("turning left")
                if (cX > 370):
                    #turn right? 
                    motors -= 200
                    turns -= 200
                    if(motors < 5000):
                        motors = 5000
                    if(turns < 5000):
                        turns = 5000
                    #tango.setTarget(MOTORS, motors)
                    #tango.setTarget(TURN, turns)
                    print("turning right")
                elif (cX < 270):
                    #turn left?
                    motors += 200
                    turns += 200
                    if(motors > 7000):
                        motors = 7000
                    if(turns > 7000):
                        turns = 7000
                    #tango.setTarget(MOTORS, motors)
                    #tango.setTarget(TURN,turns)
                    print("turning left")
                else:
                    oriented = True
                    print(oriented)
                    #default
                    motors = 6000
                    turns = 6000
                    #tango.setTarget(MOTORS, motors)
                    #tango.setTarget(TURN, turns) 
        
        cv.waitKey(1)
        if cv.waitKey(1) == 27:
            break
        
finally:
    
    # Stop streaming
    pipeline.stop()
    cv.destroyAllWindows()