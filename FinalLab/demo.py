import cv2
import pyrealsense2 as rs
import numpy as np
#from maestro import Controller

face_cascade = cv2.CascadeClassifier('data/haarcascades/haarcascade_frontalface_default.xml')

# Initialize RealSense camera
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

yellow_lower = np.array([120, 150, 150], np.uint8)
yellow_upper = np.array([200, 255, 200], np.uint8)
green_lower = np.array([140, 220, 40], np.uint8)
green_upper = np.array([180, 255,100], np.uint8)
pink_lower = np.array([150, 0, 150], np.uint8)
pink_upper = np.array([255, 100, 255], np.uint8)
orange_lower = np.array([0, 200, 20], np.uint8)
orange_upper = np.array([60, 255, 255], np.uint8)

qrCodes = ['22', '49']
progress = 0
oriented = False
miningArea = False
finishedMining = False
faceFound = False
endArea = True
colorFound = False
savedColor = None

# Set size of QR code in real world
size_of_qrcode = 0.1524 # meters


while True:
    # Get a frame from the camera
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    depth_frame = frames.get_depth_frame()
    color_image = np.asanyarray(color_frame.get_data())
    depth_image = np.asanyarray(depth_frame.get_data())
    hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
    
    if not color_frame:
        continue
    
    # Convert the frame to grayscale
    frame = cv2.cvtColor(np.array(color_frame.get_data()), cv2.COLOR_BGR2GRAY)
    
    # Detect QR codes in the frame
    if miningArea is False and finishedMining is False:
        qr_code_detector = cv2.QRCodeDetector()
        retval, decoded_info, points, straight_qrcode = qr_code_detector.detectAndDecodeMulti(frame)
        if retval is False:
            #Turn Until QRCode is found
            print("Turning Right")
            motors = 6400 # !CHANGE TO MATCH UPDATED FOLLOW LINE!
            turns = 5600 # !CHANGE TO MATCH UPDATED FOLLOW LINE!
            #tango.setTarget(MOTORS, motors)
            #tango.setTarget(TURN, turns)
        
        elif retval is True and oriented is False:
            #Put a circle at the detected QR code
            if retval is True and decoded_info[0] is qrCodes[progress]:
                center = np.mean(points[0], axis=0).astype(int)
                cv2.circle(frame, tuple(center), 5, (255, 255, 255), -1)
                text_pos = (int(center[0]), int(center[1]) - 10)
                cv2.putText(frame, str(decoded_info), text_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cX = center[0]
                cY = center[1]
                if (cX > 370):
                    #turn right? 
                    print("turning right")
                    motors = 6400 # !CHANGE TO MATCH UPDATED FOLLOW LINE!
                    turns = 5600 # !CHANGE TO MATCH UPDATED FOLLOW LINE!
                    #tango.setTarget(MOTORS, motors)
                    #tango.setTarget(TURN, turns)
                elif (cX < 270):
                    #turn left?
                    print("turning left")
                    motors = 5600 # !CHANGE TO MATCH UPDATED FOLLOW LINE!
                    turns = 6400 # !CHANGE TO MATCH UPDATED FOLLOW LINE!
                    #tango.setTarget(MOTORS, motors)
                    #tango.setTarget(TURN,turns)
                    print("turning left")
                else:
                    oriented = True
                    print("oriented")
        
        elif oriented is True:
            distance_to_qrcode = depth_frame.get_distance(cX,cY)
            if distance_to_qrcode < 1 and finishedMining is False:
                print("Now in mining area")
                miningArea = True
                finishedMining = True
                progress = 1
            elif finishedMining is True and distance_to_qrcode < 4:
                print("Now in end area")
                endArea = True
                oriented = False
            else:
                print("moving forward")
                motors = 6400 # !CHANGE TO MATCH UPDATED FOLLOW LINE!
                turns = 6400 # !CHANGE TO MATCH UPDATED FOLLOW LINE!
                #tango.setTarget(MOTORS, motors)
                #tango.setTarget(TURN,turns)
                
    if miningArea is True:
        if faceFound is False:
            gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
            faces = face_cascade.detectMultiScale(gray, 1.1, 5,)
            
            if(len(faces) == 0):
                #Turn Until Face is found
                print("Turning Right")
                motors = 6400 # !CHANGE TO MATCH UPDATED FOLLOW LINE!
                turns = 5600 # !CHANGE TO MATCH UPDATED FOLLOW LINE!
                #tango.setTarget(MOTORS, motors)
                #tango.setTarget(TURN, turns)
            elif(len(faces) != 0):
                print("Found Face!")
                for (x,y,w,h) in faces:
                    cv2.rectangle(color_image,(x,y),(x+w,y+h),(255,0,0),2)
                cX = int((x + (w/2)))
                cY = int((y + (h/2)))

                distance = depth_frame.get_distance(cX,cY)
                
                if (cX > 370):
                    #turn right? 
                    print("turning right")
                    motors = 6400 # !CHANGE TO MATCH UPDATED FOLLOW LINE!
                    turns = 5600 # !CHANGE TO MATCH UPDATED FOLLOW LINE!
                    #tango.setTarget(MOTORS, motors)
                    #tango.setTarget(TURN, turns)
                elif (cX < 270):
                    #turn left?
                    print("turning left")
                    motors = 5600 # !CHANGE TO MATCH UPDATED FOLLOW LINE!
                    turns = 6400 # !CHANGE TO MATCH UPDATED FOLLOW LINE!
                    #tango.setTarget(MOTORS, motors)
                    #tango.setTarget(TURN,turns)
                    print("turning left")
                else:
                    print("pausing")
                    motors = 6000
                    turns = 6000
                    #tango.setTarget(MOTORS, motors)
                    #tango.setTarget(TURN,turns)
                    
                if distance > 2:
                    print("moving forward")
                    motors = 6400 # !CHANGE TO MATCH UPDATED FOLLOW LINE!
                    turns = 6400 # !CHANGE TO MATCH UPDATED FOLLOW LINE!
                    #tango.setTarget(MOTORS, motors)
                    #tango.setTarget(TURN,turns)
                else:
                    faceFound = True
    
        elif faceFound is True and colorFound is False:
            if(firstLoop == True):
                motors = 6000
                turns = 6000
                #tango.setTarget(MOTORS,motors)
                #tango.setTarget(TURN,turns)
                firstLoop = False
                print("AWAITING ICE")
                for x in range(5000000):
                    pass
                
            if(savedColor == None):
                yellow_mask = cv2.inRange(color_image, yellow_lower, yellow_upper)
  
                green_mask = cv2.inRange(color_image, green_lower, green_upper)
  
                pink_mask = cv2.inRange(color_image, pink_lower, pink_upper)

                kernel = np.ones((5, 5), "uint8")
      
                yellow_mask = cv2.dilate(yellow_mask, kernel)
                res_yellow = cv2.bitwise_and(color_image, color_image, mask = yellow_mask)
      
                green_mask = cv2.dilate(green_mask, kernel)
                res_green = cv2.bitwise_and(color_image, color_image, mask = green_mask)
      
                pink_mask = cv2.dilate(pink_mask, kernel)
                res_pink = cv2.bitwise_and(color_image, color_image, mask = pink_mask)
   
                contours, hierarchy = cv2.findContours(yellow_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
                for pic, contour in enumerate(contours):
                    area = cv2.contourArea(contour)
                    if(area > 1000):
                        savedColor = "yellow"
                        x, y, w, h = cv2.boundingRect(contour)
                        color_image = cv2.rectangle(color_image, (x, y), (x + w, y + h), (51, 255, 255), 2)
                

                # Creating contour to track green color
                contours, hierarchy = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
      
                for pic, contour in enumerate(contours):
                    area = cv2.contourArea(contour)
                    if(area > 1000):
                        savedColor = "green"
                        x, y, w, h = cv2.boundingRect(contour)
                        color_image = cv2.rectangle(color_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                

                contours, hierarchy = cv2.findContours(pink_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            
                for pic, contour in enumerate(contours):
                    area = cv2.contourArea(contour)
                    if(area > 1000):
                        savedColor = "pink"
                        x, y, w, h = cv2.boundingRect(contour)
                        color_image = cv2.rectangle(color_image, (x, y), (x + w, y + h), (255, 77, 255), 2)
                        
            if(savedColor != None):
                print("COLOR DETECTED: " + savedColor)
                
        if endArea is True:
            if(savedColor == "yellow"):
                color_mask = cv2.inRange(color_image, yellow_lower, yellow_upper)

            if(savedColor == "green"):
                color_mask = cv2.inRange(color_image, green_lower, green_upper)
  
            if(savedColor == "pink"):
                color_mask = cv2.inRange(color_image, pink_lower, pink_upper)
                
            Moments = cv2.moments(color_mask)
            if Moments["m00"] != 0:
                cX = int(Moments["m10"] / Moments["m00"])
                cY = int(Moments["m01"] / Moments["m00"])
            else:
                cX, cY = 0,0
            cv2.circle(color_image, (cX, cY), 5, (0, 165, 255), -1)

            distance = depth_frame.get_distance(cX,cY)
                
            if (cX > 370):
                #turn right? 
                print("turning right")
                motors = 6400 # !CHANGE TO MATCH UPDATED FOLLOW LINE!
                turns = 5600 # !CHANGE TO MATCH UPDATED FOLLOW LINE!
                #tango.setTarget(MOTORS, motors)
                #tango.setTarget(TURN, turns)
            elif (cX < 270):
                #turn left?
                print("turning left")
                motors = 5600 # !CHANGE TO MATCH UPDATED FOLLOW LINE!
                turns = 6400 # !CHANGE TO MATCH UPDATED FOLLOW LINE!
                #tango.setTarget(MOTORS, motors)
                #tango.setTarget(TURN,turns)
                print("turning left")
            elif distance > 0.5:
                print("moving forward")
                motors = 6400 # !CHANGE TO MATCH UPDATED FOLLOW LINE!
                turns = 6400 # !CHANGE TO MATCH UPDATED FOLLOW LINE!
                #tango.setTarget(MOTORS, motors)
                #tango.setTarget(TURN,turns)
            else:
                print("done")
                break
    
    # Display the frame
    cv2.imshow('QR Code Detector', frame)
    
    # Exit if the 'q' key is pressed
    if cv2.waitKey(1) == 27:
        break

# Clean up
cv2.destroyAllWindows()
pipeline.stop()