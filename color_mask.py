import numpy as np
import cv2 as cv
from opencv_utils import *

def color_mask(image_bga: np.ndarray, low_hsv, high_hsv, erosion_iterations = 2)-> np.ndarray:
    image_hsv = cv.cvtColor(image_bga, cv.COLOR_BGR2HSV)
    mask = cv.inRange(image_hsv, low_hsv, high_hsv)
    mask = erode(mask, erosion_iterations)
    return mask

# def orange_mask(image: np.ndarray) -> np.ndarray:
#     image_hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
#     orange = np.array([31,98,255], np.uint8)
#     orange_low = np.array([31 - 20,50,150], np.uint8)
#     orange_high = np.array([31 + 20, 255, 230], np.uint8)
#
#     mask = cv.inRange(image_hsv, orange_low, orange_high)
#     return mask
#
#     # return cv.bitwise_and(image, image, mask=mask)
