import numpy as np
import pyrealsense2 as rs
import cv2

# Set up Realsense pipeline and config
pipeline = rs.pipeline()
config = rs.config()

# Enable the stream of color and depth
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# Start streaming
pipeline.start(config)

# Define ArUco dictionary and parameters
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters_create()

while True:
    # Wait for a new frame
    frames = pipeline.wait_for_frames()

    # Get the color and depth frames
    color_frame = frames.get_color_frame()
    depth_frame = frames.get_depth_frame()

    # Convert color frame to OpenCV format
    color_image = np.asanyarray(color_frame.get_data())

    # Convert depth frame to OpenCV format
    depth_image = np.asanyarray(depth_frame.get_data())

    # Detect ArUco markers in the color image
    corners, ids, rejected = cv2.aruco.detectMarkers(color_image, aruco_dict, parameters=aruco_params)

    # If any markers are detected
    if ids is not None:
        # Print out the detected IDs
        print(ids)

        # Draw the markers on the color image
        cv2.aruco.drawDetectedMarkers(color_image, corners, ids)

    # Display the color image
    cv2.imshow("Color Image", color_image)

    # Wait for a key press
    key = cv2.waitKey(1)

    # If the 'q' key is pressed, exit the loop
    if key == ord('q'):
        break

# Stop streaming
pipeline.stop()

# Close all OpenCV windows
cv2.destroyAllWindows()