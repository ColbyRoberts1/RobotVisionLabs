import cv2
import time
import pyrealsense2 as rs
import numpy as np
import cv2.aruco as aruco
from maestro import Controller

def move(direction, moveDuration, waitTime):
    if direction is 'left':
        tango.setTarget(MOTORS, 7000)
        tango.setTarget(TURN, 7000)
    if direction is 'forward':
        tango.setTarget(MOTORS, 5000)
        tango.setTarget(TURN, 7000)
    if direction is 'right':
        tango.setTarget(MOTORS, 5000)
        tango.setTarget(TURN, 5000)
    if direction is 'backward':
        tango.setTarget(MOTORS, 7000)
        tango.setTarget(TURN, 5000)
    time.sleep(moveDuration)
    tango.setTarget(MOTORS, 6000)
    tango.setTarget(TURN, 6000)
    time.sleep(waitTime)

face_cascade = cv2.CascadeClassifier('data/haarcascades/haarcascade_frontalface_default.xml')

# Initialize RealSense camera
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipeline.start(config)
frames = pipeline.wait_for_frames()
tango = Controller()

aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
parameters = aruco.DetectorParameters_create()

MOTORS = 1
TURN = 2
BODY = 0
HEADTILT = 4

tango.setTarget(HEADTILT, 6000)

yellow_lower = np.array([0, 51, 178], np.uint8)
yellow_upper = np.array([57, 218, 256], np.uint8)
green_lower = np.array([0, 85, 181], np.uint8)
green_upper = np.array([256, 150, 249], np.uint8)
pink_lower = np.array([52, 132, 186], np.uint8)
pink_upper = np.array([251, 206, 230], np.uint8)
motors = 6000

qrCodes = ['22', '49']
progress = 0
miningArea = False
finishedMining = False
faceFound = False
endArea = True
colorFound = False
savedColor = None
atEnd = False 
firstLoop = True

# Set size of QR code in real world
size_of_qrcode = 0.1524 # meters

while True:
    # Get a frame from the camera
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    depth_frame = frames.get_depth_frame()
    color_image = np.asanyarray(color_frame.get_data())
    depth_image = np.asanyarray(depth_frame.get_data())
    hsvcolor_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
    if not color_frame:
        continue
    
    # Convert the frame to grayscale
    frame = cv2.cvtColor(np.array(color_frame.get_data()), cv2.COLOR_BGR2GRAY)
    
    # Journey towards orange cone
    if miningArea is False:
        #qr_code_detector = cv2.QRCodeDetector()
        #retval, decoded_info, points, straight_qrcode = qr_code_detector.detectAndDecodeMulti(frame)
        c = 0
        corners, ids, reject = aruco.detectMarkers(frame, aruco_dict, parameters=parameters,)
        if len(corners) > 0:
            ids = ids.flatten()
            for (markerCorner, markerID) in zip(corners, ids):
                if (markerID == np.int32(49)):
                    corners = markerCorner.reshape((4, 2))
                    (tl, tr, br, bl) = corners
                    tr = (int(tr[0]), int(tr[1]))
                    tl = (int(tl[0]), int(tl[1]))
                    br = (int(br[0]), int(br[1]))
                    bl = (int(bl[0]), int(bl[1]))
                    cX = int((tl[0] + br[0]) / 2)
                    cY = int((tl[1] + br[1]) / 2)
                    c = 1

        if c is 0:
            move('right', .5, 1)
        
        elif c is 1:
            cv2.circle(color_image, (cX, cY), 5, (0, 165, 255), -1)
            distance = depth_frame.get_distance(cX,cY)

            if (cX > 370):
                move('right', .2, 1)
            elif (cX < 270):
                move('left', .2, 1)
            else:
                if(distance > 1):
                    move('forward', .75, 1)
            
                elif(distance <= 1):
                    tango.setTarget(MOTORS, 6000)
                    tango.setTarget(TURN, 6000)
                    print("Entered Mining Area!")
                    miningArea = True
                
    elif miningArea is True:
        if faceFound is False:
            gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
            faces = face_cascade.detectMultiScale(gray, 1.1, 5,)
            
            if(len(faces) == 0):
                #Turn Until Face is found
                move('right', .25, 1.5)
            elif(len(faces) != 0):
                print("Found Face!")
                for (x,y,w,h) in faces:
                    cv2.rectangle(color_image,(x,y),(x+w,y+h),(255,0,0),2)
                cX = int((x + (w/2)))
                cY = int((y + (h/2)))

                distance = depth_frame.get_distance(cX,cY)
                
                if (cX > 370):
                    #turn right? 
                    move('right', .2, 1)
                elif (cX < 270):
                    #turn left?
                    move('left', .2, 1)
                else:
                    print("pausing")
                    motors = 6000
                    turns = 6000
                    #tango.setTarget(MOTORS, motors)
                    #tango.setTarget(TURN,turns)
                    
                if distance > 1:
                    move('forward', .4, 1)
                else:
                    faceFound = True
    
        elif faceFound is True and savedColor is None:
            if(firstLoop == True):
                motors = 6000
                turns = 6000
                #tango.setTarget(MOTORS,motors)
                #tango.setTarget(TURN,turns)
                firstLoop = False
                print("AWAITING ICE")
                
            
            yellow_mask = cv2.inRange(hsvcolor_image, yellow_lower, yellow_upper)

            green_mask = cv2.inRange(hsvcolor_image, green_lower, green_upper)

            pink_mask = cv2.inRange(hsvcolor_image, pink_lower, pink_upper)

            kernel = np.ones((5, 5), "uint8")
    
            yellow_mask = cv2.dilate(yellow_mask, kernel)
            res_yellow = cv2.bitwise_and(hsvcolor_image, hsvcolor_image, mask = yellow_mask)
    
            green_mask = cv2.dilate(green_mask, kernel)
            res_green = cv2.bitwise_and(hsvcolor_image, hsvcolor_image, mask = green_mask)
    
            pink_mask = cv2.dilate(pink_mask, kernel)
            res_pink = cv2.bitwise_and(hsvcolor_image, hsvcolor_image, mask = pink_mask)

            contours, hierarchy = cv2.findContours(yellow_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
            for pic, contour in enumerate(contours):
                area = cv2.contourArea(contour)
                if(area > 1000):
                    savedColor = "yellow"
                    print(savedColor)
                    x, y, w, h = cv2.boundingRect(contour)
                    color_image = cv2.rectangle(hsvcolor_image, (x, y), (x + w, y + h), (51, 255, 255), 2)
            

            # Creating contour to track green color
            contours, hierarchy = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
            for pic, contour in enumerate(contours):
                area = cv2.contourArea(contour)
                if(area > 1000):
                    savedColor = "green"
                    print(savedColor)
                    x, y, w, h = cv2.boundingRect(contour)
                    color_image = cv2.rectangle(hsvcolor_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            

            contours, hierarchy = cv2.findContours(pink_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
            for pic, contour in enumerate(contours):
                area = cv2.contourArea(contour)
                if(area > 1000):
                    savedColor = "pink"
                    print(savedColor)
                    x, y, w, h = cv2.boundingRect(contour)
                    color_image = cv2.rectangle(hsvcolor_image, (x, y), (x + w, y + h), (255, 77, 255), 2)
        elif savedColor is not None and atEnd is False:
            c = 0
            corners, ids, reject = aruco.detectMarkers(frame, aruco_dict, parameters=parameters,)
            if len(corners) > 0:
                ids = ids.flatten()
                for (markerCorner, markerID) in zip(corners, ids):
                    if (markerID == np.int32(22)):
                        corners = markerCorner.reshape((4, 2))
                        (tl, tr, br, bl) = corners
                        tr = (int(tr[0]), int(tr[1]))
                        tl = (int(tl[0]), int(tl[1]))
                        br = (int(br[0]), int(br[1]))
                        bl = (int(bl[0]), int(bl[1]))
                        cX = int((tl[0] + br[0]) / 2)
                        cY = int((tl[1] + br[1]) / 2)
                        c = 1

            if c is 0:
                move('right', .5, .5)

            elif c is 1:
                cv2.circle(color_image, (cX, cY), 5, (0, 165, 255), -1)
                distance = depth_frame.get_distance(cX,cY)

                if (cX > 370):
                    move('right', .2, 1)
                elif (cX < 270):
                    move('left', .2, 1)
                else:
                    if(distance > 2.5):
                        move('forward', .75, 1)

                    elif(distance <= 2.5):
                        tango.setTarget(MOTORS, 6000)
                        tango.setTarget(TURN, 6000)
                        print("At End Area!")
                        atEnd = True
                        
        elif savedColor is not None and atEnd is True:
            print("COLOR DETECTED: " + savedColor)

            tango.setTarget(HEADTILT, 4500)

            if(savedColor == "yellow"):
                color_mask = cv2.inRange(color_image, yellow_lower, yellow_upper)

            elif(savedColor == "green"):
                color_mask = cv2.inRange(color_image, green_lower, green_upper)

            elif(savedColor == "pink"):
                color_mask = cv2.inRange(color_image, pink_lower, pink_upper)
                
            Moments = cv2.moments(color_mask)
            if Moments["m00"] != 0:
                cX = int(Moments["m10"] / Moments["m00"])
                cY = int(Moments["m01"] / Moments["m00"]) 
                
                if (cX > 370):
                    move('right', .2, 1)
                elif (cX < 270):
                    #turn left?
                    move('left', .2, 1)
                elif distance > 1:
                    print("moving forward")
                    move('forward', .4, 1)
                else:
                    tango.setTarget(MOTORS, 6000)
                    tango.setTarget(TURN, 6000)
                    print("Done!")
                    break
            
            else:
                move('left', .3, 1)
        
    cv2.imshow("color", color_image)

    # Exit if the 'q' key is pressed
    if cv2.waitKey(1) == 27:
        break

        
tango.setTarget(MOTORS, 6000)
tango.setTarget(TURN, 6000)

# Clean up
cv2.destroyAllWindows()
pipeline.stop()
