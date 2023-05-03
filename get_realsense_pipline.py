import numpy as np
import pyrealsense2 as rs
import cv2 as cv




def get_realsense_pipeline():
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()

    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))

    # camera setup
    found_rgb = False
    for s in device.sensors:
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            found_rgb = True
            break
    if not found_rgb:
        exit(1)

    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    if device_product_line == 'L500':
        config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
    else:
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    profile = pipeline.start(config)

    # get depth sensor's scale
    # depth_sensor = profile.get_device().first_depth_sensor()
    # depth_scale = depth_sensor.get_depth_scale()

    return pipeline


def next_bgr_frame(pipeline) -> np.ndarray:
    color_frame = None
    while not color_frame:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

    # Convert image to numpy arrays
    image: np.ndarray = np.asanyarray(color_frame.get_data())
    # image = cv.resize(image, (320, 240))
    return image
