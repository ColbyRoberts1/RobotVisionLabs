import cv2 as cv

# for keeping track of trackbars
class Trackbar:
    def __init__(self, window_name: str, trackbar_name: str, max_value: int, array=None, array_index: int = None,
                 initial_value: int = None, on_change=None):
        self.trackbar_name = trackbar_name
        self.window_name = window_name

        if initial_value is not None:
            self.initial_value = initial_value
        elif array is not None and array_index is not None:
            self.initial_value = array[array_index]
        else:
            self.initial_value = 0

        self.max_value = max_value
        self.value = self.initial_value
        self.on_change = on_change
        self.array = array
        self.array_index = array_index

        def self_on_change(new_value):
            self.value = new_value

            if self.array is not None and self.array_index is not None:
                self.array[array_index] = new_value

            if self.on_change is not None:
                self.on_change(new_value)

        cv.createTrackbar(self.trackbar_name, self.window_name, self.initial_value, self.max_value, self_on_change)
