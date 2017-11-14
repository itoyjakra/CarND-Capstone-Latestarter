from styx_msgs.msg import TrafficLight
import numpy as np
import cv2

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier

        self.colors = ['red', 'yellow', 'green']
        self.sensitivity = 15
        self.mask = dict()

    def create_masks(self, cv_image):
        """
        Create masks for the three colors of traffic light
        """

        # convert image to HSV for better color detection
        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # create a red mask
        lower_red = np.array([0,100,100])
        upper_red = np.array([10,255,255])
        mask1 = cv2.inRange(hsv_img, lower_red, upper_red)
        lower_red = np.array([160,100,100])
        upper_red = np.array([179,255,255])
        mask2 = cv2.inRange(hsv_img, lower_red, upper_red)
        self.mask['red'] = cv2.bitwise_or(mask1, mask2)

        # create a green mask
        lower_green = np.array([60 - self.sensitivity, 100, 100])
        upper_green = np.array([60 + self.sensitivity, 255, 255])
        self.mask['green'] = cv2.inRange(hsv_img, lower_green, upper_green)

        # create a yellow mask
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])
        self.mask['yellow'] = cv2.inRange(hsv_img, lower_yellow, upper_yellow)

    def detect_light(self, image, color):
        """
        Detects the existence of a traffic light of given color
        and returns center and radius of the light in the image
        """

        #assert color in self.colors
        res = cv2.bitwise_and(image, image, mask = self.mask[color])
        res_bgr = cv2.cvtColor(res, cv2.COLOR_HSV2BGR)
        res_mono = cv2.cvtColor(res_bgr, cv2.COLOR_BGR2GRAY)
        circles = cv2.HoughCircles(res_mono, cv2.HOUGH_GRADIENT, dp=1, minDist=20, minRadius=5, param1=1, param2=20)

        return circles

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        # Create color masks
        self.create_masks(image)

        # get the detected lights for the three colors
        for color in self.colors:
            light_circles = self.detect_light(image, color)
            if light_circles is not None:
                print('detected color = ', color)
                return self.colors.index(color)

        return TrafficLight.UNKNOWN
