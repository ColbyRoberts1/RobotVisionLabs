import cv2
import numpy as np
from typing import Callable
import cv2 as cv
from common import *


# gets the number of non-black pixels in the binary image
def non_black_count(image: np.ndarray) -> int:
    x_coords, y_coords = image.nonzero()
    return len(x_coords)


# gets the ratio of non-black to black in the binary image
def non_black_ratio(image: np.ndarray) -> float:
    res = image.shape[0] * image.shape[1]
    count = non_black_count(image)
    return count / res


# get center of gravity of non-black pixels in binary image
def get_cog(image: np.ndarray):
    x_vals, y_vals = image.nonzero()
    if len(x_vals) == 0:
        return [rint(image.shape[1] / 2), rint(image.shape[0] / 2)]
    cog = [rint(float(sum(x_vals)) / len(x_vals)), rint(float(sum(y_vals)) / len(y_vals))]

    return cog


# fits a line to the non-black pixels in the binary image
def fitLine(binary_image: np.ndarray):
    # make the image more manageable (the original one was lagging on my ryzen 7)
    smaller_binary_image = cv.resize(binary_image, (50, 50))

    # get coords of all the white pixels as array of [x, y]
    x_points, y_points = smaller_binary_image.nonzero()
    points = np.stack((x_points, y_points), axis=1)

    # incase there were no white pixels in the image (horizontal line)
    if points.shape[0] == 0:
        return [[1, 0], [0, 0]]

    line = cv.fitLine(points, cv2.DIST_L1, 0, 0.01, 0.01) \
        .flatten() \
        .reshape((2, 2))
    # ^^ format output a little better ^^, [[v0, v1],[x, y]]

    # flip vector so it's [x, y]
    temp = line[0][0]
    line[0][0] = line[0][1]
    line[0][1] = temp

    return line[0], line[1]


def erode(image: np.ndarray, iterations) -> np.ndarray:
    kernel = np.ones((2, 2), np.uint8)
    return cv.erode(image, kernel, iterations=iterations)


# these two functions are useless because of how stupid slow python is (12 seconds to run!)

def get_top_left_most_pixel(image: np.ndarray, predicate: Callable[[np.ndarray, int, int], bool]) -> (
        int, int, np.ndarray):
    height, width = image.shape[:2]
    for x in range(0, width):
        for y in range(0, height):
            pixel = image[y][x]
            if predicate(pixel, x, y):
                return x, y, pixel

    return None


def get_top_right_most_pixel(image: np.ndarray, predicate: Callable[[np.ndarray, int, int], bool]) -> (
        int, int, np.ndarray):
    height, width = image.shape[:2]
    for x in range(width - 1, -1, -1):
        for y in range(0, height):
            pixel = image[y][x]
            if predicate(pixel, x, y):
                return x, y, pixel

    return None
