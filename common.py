import math
import time
import cv2 as cv


def current_milli_time():
    return round(time.time() * 1000)


# round to an int
def rint(num) -> int:
    return int(round(num))


# python doesn't have a sign function?!
def sign(num) -> int:
    if num < 0:
        return -1
    if num > 0:
        return 1
    return 0


# applies an aggressive log curve to the number (shoots up for lower values, plateaus on higher values)
def log_curve(num) -> float:
    return (math.log2(abs(num) * 10 + 1) * 0.29) * sign(num)


# applies a x^2 curve to the number
def square_curve(num) -> float:
    return num * num * sign(num)


def waitKey(milliseconds):
    key = cv.waitKey(milliseconds) & 0xff
    # if esc
    if key == 27:
        raise Exception("escape pressed")
        # controller.setTarget(1, 6000)
        # controller.setTarget(2, 6000)
        #
        # # kill the whole program
        # exit(0)
