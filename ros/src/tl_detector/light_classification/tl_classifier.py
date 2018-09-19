from styx_msgs.msg import TrafficLight
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import numpy as np

class TLClassifier(object):
    def __init__(self):
        self.image_pub1 = rospy.Publisher("tl_debug/ch1",Image, queue_size=1)
        self.image_pub2 = rospy.Publisher("tl_debug/ch2",Image, queue_size=1)
        self.image_pub3 = rospy.Publisher("tl_debug/ch3",Image, queue_size=1)
        self.debug_im1 = None
        self.debug_im2 = None
        self.debug_im3 = None
        self.bridge = CvBridge()


    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        IN HSV Space, light colors are generally:

        RED:    240 < H < 255
                195 < S < 215
                  0 < V < 5

        Yellow: 240 < H < 255
                195 < S < 215
                28  < V < 35

        Green:  240 < H < 255
                195 < S < 215
                 60 < V < 67

        """

        temp = cv2.cvtColor(cv2.GaussianBlur(image,(5,5),0), cv2.COLOR_BGR2HSV)

        maskR = cv2.inRange(temp, np.array([0, 195, 240]), np.array([5, 215, 255]))
        maskY = cv2.inRange(temp, np.array([28, 195, 240]), np.array([35, 215, 255]))
        maskG = cv2.inRange(temp, np.array([60, 195, 240]), np.array([67, 215, 255]))

        filt_r = cv2.bitwise_and(temp,temp, mask= maskR)
        filt_y = cv2.bitwise_and(temp,temp, mask= maskY)
        filt_g = cv2.bitwise_and(temp,temp, mask= maskG)

        # Bitwise-AND mask and original image
        self.debug_im1 = filt_r
        self.debug_im2 = filt_y
        self.debug_im3 = filt_g
        status = TrafficLight.UNKNOWN

        if np.sum(maskR>10):
            print('detected red')
            status = TrafficLight.RED
        elif np.sum(maskY>10):
            print('detected yellow')
            status = TrafficLight.YELLOW
        elif np.sum(maskG>10):
            print('detected green')
            status = TrafficLight.GREEN

        # self.debug()
        return status


    def debug(self):
        if self.debug_im1 is not None:
            self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.debug_im1, "bgr8"))
        if self.debug_im2 is not None:
            self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.debug_im2, "bgr8"))
        if self.debug_im3 is not None:
            self.image_pub3.publish(self.bridge.cv2_to_imgmsg(self.debug_im3, "bgr8"))