from styx_msgs.msg import TrafficLight

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier

        self.mask = dict()
        # create a red mask
        lower_red = np.array([0,100,100])
        upper_red = np.array([10,255,255])
        mask1 = cv2.inRange(hsv, lower_red, upper_red)
        lower_red = np.array([160,100,100])
        upper_red = np.array([179,255,255])
        mask2 = cv2.inRange(hsv, lower_red, upper_red)
        self.mask['red'] = cv2.bitwise_or(mask1, mask2)

        # create a green mask
        sensitivity = 15
        lower_green = [60 - sensitivity, 100, 100]
        upper_green = [60 + sensitivity, 255, 255]
        mask2 = cv2.inRange(hsv, lower_red, upper_red)
        self.mask['green'] = cv2.bitwise_or(mask1, mask2)

        # create a yellow mask
        self.mask['yellow'] = cv2.bitwise_or(mask1, mask2)

    def detect_light(self, image, color):
        """
        Detects the existence of a traffic light of given color
        and returns center and radius of the light in the image
        """
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
        # convert image to HSV for better color detection
        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # get the detected lights for the three colors
        for color in self.colors:
            light_circles = self.detect_light(hsv_img, color)
            if light_circles is not None:
                return color_index

        return None

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")




            #print('number of circles = {}'.format(circles.shape))
            if circles is not None:
                circles = np.uint16(np.around(circles))
                for index, i in enumerate(circles[0,:]):
                    # draw the outer circle
                    #cv2.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
                    print('circle: ', index, i[0], i[1], i[2])
            #print('max, min of red = {}, {}'.format(np.amax(red), np.amin(red)))
            #print('argmax0, argmax1 of red = {}, {}'.format(np.argmax(red, axis=0), np.argmax(red, axis=1)))
            #print(len(np.argmax(red, axis=0)))
            #if np.amax(red) > 250:
            #    print('red light detected')
            #else:
            #    print('no red light')
        except CvBridgeError as e:
            print(e)
        #cv2.imshow("Image window", cv_image)
        cv2.imshow("Image window", res_mono)
        cv2.waitKey(3)
        return TrafficLight.UNKNOWN
