from color_mask import *
from get_realsense_pipline import *
from opencv_utils import *
from trackbar import *


def main():
    pipeline = get_realsense_pipeline()
    cv.namedWindow("hsv values", cv.WINDOW_AUTOSIZE)
    color_hsv_low = np.array([0, 0, 0])
    color_hsv_high = np.array([256, 256, 256])
    erosion = [0]

    Trackbar("hsv values", "low  hue", 256, color_hsv_low, 0)
    Trackbar("hsv values", "low  sat", 256, color_hsv_low, 1)
    Trackbar("hsv values", "low  val", 256, color_hsv_low, 2)

    Trackbar("hsv values", "high hue", 256, color_hsv_high, 0)
    Trackbar("hsv values", "high sat", 256, color_hsv_high, 1)
    Trackbar("hsv values", "high val", 256, color_hsv_high, 2)
    Trackbar("hsv values", "erosion ", 10, erosion, 0)
    while True:
        image = next_bgr_frame(pipeline)
        image = cv.resize(image, (320, 240))
        height = image.shape[0]
        width = image.shape[1]

        print(image.shape)

        # color mask
        mask = color_mask(image, color_hsv_low, color_hsv_high, erosion[0])
        # mask = erode(mask, erosion[0])
        mask_color = cv.cvtColor(mask, cv.COLOR_GRAY2BGR)

        white_ratio = non_black_ratio(mask)
        # print("ratio: " + str(white_ratio))

        # draw status text
        cv.putText(image, "ratio: " + str(white_ratio), (50, 50), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0))

        cv.imshow("image and mask", np.hstack((image, mask_color)))

        waitKey(1)


main()
