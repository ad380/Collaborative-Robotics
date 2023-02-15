#!/usr/bin/env python3
import rospy
from cv_bridge import CvBridge
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage, Image

class CubeDetector:

    def __init__(self) -> None:
        sub = rospy.Subscriber("/locobot/camera/color/image_raw/compressed", CompressedImage, self.camera_callback)
        self.pub = rospy.Publisher("/locobot/cubes/compressed", CompressedImage, queue_size=1)

    def detect_cube(self, color_img, color_mask='r'):
        """Returns the black and white image with a color-mask for the specified color (white or the color where the color is, black everywhere else)
        
        Parameters
        ----------
        color_img : np.ndarray
            Raw input image of colored blocks on a table
        color_mask : string
            String indicating which color to draw the mask on; options 'r', 'g','b','y' (red, green, blue, yellow)
        
        Returns
        -------
        mask_img : np.ndarray
            Image with just the selected color shown (either with true color or white mask)
        """    
        # cv2.imwrite("raw_image.jpg", color_img)
        
        #Step 1: Convert to HSV space; OpenCV uses - H: 0-179, S: 0-255, V: 0-255
        hsv_img = cv2.cvtColor(color_img, cv2.COLOR_BGR2HSV)
        # blur = cv2.GaussianBlur(hsv_img, (5, 5), 0)

        #Step 2: prep the mask
        if color_mask == 'r':
            low_H = 0
            high_H = 12
        elif color_mask == 'g':
            low_H = 35
            high_H = 85
        elif color_mask == 'b':
            low_H = 90
            high_H = 135
        elif color_mask == 'y':
            low_H = 20
            high_H = 50

        mask = cv2.inRange(hsv_img, (0, 0, 0), (255, 135, 255))
        mask = cv2.bitwise_not(mask)
        color_img = cv2.bitwise_and(color_img, color_img, mask=mask)
        cv2.imwrite("test.jpg", color_img)

        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = sorted(contours, key=lambda c: cv2.contourArea(c))[-10:]
        area = list(map(cv2.contourArea, contours))
        peri = list(map(lambda c: cv2.arcLength(c, True), contours))
        ratio = np.divide(area, peri)
        idx = np.argpartition(ratio, -min(len(ratio), 4))[-4:]
        filtered = np.array(contours)[idx]
        mask = np.zeros_like(mask)
        mask = cv2.drawContours(mask, filtered, -1, 255, cv2.FILLED)
        hsv_img = cv2.bitwise_and(hsv_img, hsv_img, mask=mask)

        #Step 3: Apply the mask; black region in the mask is 0, so when multiplied with original image removes all non-selected color 
        mask = cv2.inRange(hsv_img, (low_H, 0, 20), (high_H, 255, 255)) 
        img = cv2.bitwise_and(color_img, color_img, mask=mask)    
        
        return img

        
    def camera_callback(self, msg: CompressedImage):
        bridge = CvBridge()
        image = bridge.compressed_imgmsg_to_cv2(msg)
        r = self.detect_cube(image, 'r')
        g = self.detect_cube(image, 'g')
        b = self.detect_cube(image, 'b')
        y = self.detect_cube(image, 'y')
        big = np.hstack([
            np.vstack([r, b]),
            np.vstack([g, y])
        ])
        # big_msg = bridge.cv2_to_imgmsg(big)
        big_msg = bridge.cv2_to_compressed_imgmsg(big)
        # print(big.shape)
        self.pub.publish(big_msg)

def main():
    rospy.init_node('cube_detection')
    detector = CubeDetector()
    rospy.spin()


if __name__ == "__main__":
    main()

