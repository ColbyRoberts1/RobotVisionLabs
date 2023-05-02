import cv2
import pyrealsense2 as rs
import numpy as np

# Initialize RealSense camera
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

while True:
    # Get a frame from the camera
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    if not color_frame:
        continue
    
    # Convert the frame to grayscale
    frame = cv2.cvtColor(np.array(color_frame.get_data()), cv2.COLOR_BGR2GRAY)
    
    # Detect QR codes in the frame
    qr_code_detector = cv2.QRCodeDetector()
    retval, decoded_info, points, straight_qrcode = qr_code_detector.detectAndDecodeMulti(frame)
    
    # Put a circle at the detected QR code
    if retval is True:
        center = np.mean(points[0], axis=0).astype(int)
        cv2.circle(frame, tuple(center), 5, (255, 255, 255), -1)
        text_pos = (int(center[0]), int(center[1]) - 10)
        cv2.putText(frame, str(decoded_info), text_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    # Display the frame
    cv2.imshow('QR Code Detector', frame)
    
    # Exit if the 'q' key is pressed
    if cv2.waitKey(1) == 27:
        break
# Clean up
cv2.destroyAllWindows()
pipeline.stop()