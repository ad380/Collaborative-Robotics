#!/usr/bin/env python3
import rospy
from cv_bridge import CvBridge
import numpy as np
import cv2
import tf
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from image_geometry import PinholeCameraModel

from interbotix_xs_modules.locobot import InterbotixLocobotXS


class CubeDetector:

    def __init__(self) -> None:
        self.pub = rospy.Publisher("/locobot/cubes/compressed", CompressedImage, queue_size=1)
        self.bridge = CvBridge()
        self.camera_model = PinholeCameraModel()
        self.listener = tf.TransformListener()

        self.camera_cube_locator_marker = rospy.Publisher("/locobot/camera_cube_locator", Marker, queue_size=1)
        self.point_3d = PointStamped()
        self.centers = []
        self.locobot = InterbotixLocobotXS(robot_model="locobot_wx250s", arm_model="mobile_wx250s", use_move_base_action=False, init_node=False)

        self.sub = rospy.Subscriber("/locobot/camera/color/image_raw/compressed", CompressedImage, self.camera_callback)
        self.info_sub = rospy.Subscriber("/locobot/camera/aligned_depth_to_color/camera_info", CameraInfo, self.info_callback)
        rospy.sleep(2)
        self.depth_sub = rospy.Subscriber('/locobot/camera/aligned_depth_to_color/image_raw', Image, self.depth_callback)


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
        cv2.imwrite("raw_image.jpg", color_img)
        
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

        mask = cv2.inRange(hsv_img, (0, 0, 0), (255, 110, 255))
        mask = cv2.bitwise_not(mask)
        color_img = cv2.bitwise_and(color_img, color_img, mask=mask)
        cv2.imwrite("test.jpg", color_img)

        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # contours = sorted(contours, key=lambda c: cv2.contourArea(c))[-10:]
        # area = np.asarray(list(map(cv2.contourArea, contours)))
        # peri = np.asarray(list(map(lambda c: cv2.arcLength(c, True), contours)))
        # ratio = area / peri
        # # idx = np.argpartition(ratio, -min(len(ratio), 4))[-4:]
        # # idx = area > 100
        # filtered = np.array(contours)[idx]
        mask = np.zeros_like(mask)
        mask = cv2.drawContours(mask, contours, -1, 255, cv2.FILLED)
        hsv_img = cv2.bitwise_and(hsv_img, hsv_img, mask=mask)

        #Step 3: Apply the mask; black region in the mask is 0, so when multiplied with original image removes all non-selected color 
        mask = cv2.inRange(hsv_img, (low_H, 0, 20), (high_H, 255, 255)) 

        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        area = np.asarray(list(map(cv2.contourArea, contours)))
        contours = np.asarray(contours)[area > 100]
        mask = np.zeros_like(mask)
        mask = cv2.drawContours(mask, contours, -1, 255, cv2.FILLED)
        # print(f'Color {color_mask}, num contours: {len(contours)}')

        img = cv2.bitwise_and(color_img, color_img, mask=mask)    
        
        centers = []
        for cont in contours:
            center = np.mean(cont, axis=0)[0].astype(int)
            img = cv2.circle(img, tuple(center), 3, (255, 255, 255), -1)
            centers.append(center)

        return img, np.array(centers)

        
    def camera_callback(self, msg: CompressedImage):
        image = self.bridge.compressed_imgmsg_to_cv2(msg)
        r, r_centers = self.detect_cube(image, 'r')
        g, g_centers = self.detect_cube(image, 'g')
        b, b_centers = self.detect_cube(image, 'b')
        y, y_centers = self.detect_cube(image, 'y')
        big = np.hstack([
            np.vstack([r, b]),
            np.vstack([g, y])
        ])
        big_msg = self.bridge.cv2_to_compressed_imgmsg(big)

        self.centers = []
        self.colors = []
        for c, points in zip(['r', 'g', 'b', 'y'], [r_centers, g_centers, b_centers, y_centers]):
            for p in points:
                self.centers.append(p)
                self.colors.append(c)
        self.centers = np.asarray(self.centers)
        # print(self.centers)

        self.pub.publish(big_msg)

    def depth_callback(self, msg: Image):
        depth_image = self.bridge.imgmsg_to_cv2(msg)

        min_depth = float('inf')
        min_point = None
        for (y, x) in self.centers:
            depth = depth_image[x, y]
            if depth < min_depth:
                min_depth = depth
                min_point = (y, x)
        
        # print(min_point, min_depth)

        if min_point is not None:
            # use the camera model to get the 3D ray for the current pixel
            ray = self.camera_model.projectPixelTo3dRay(min_point)

            # calculate the 3D point on the ray using the depth value
            point_3d = np.array(ray) * min_depth / 1000
            # print(point_3d)

            point_3d_geom_msg = PointStamped()
            point_3d_geom_msg.header = msg.header
            point_3d_geom_msg.header.stamp = rospy.Time(0)
            point_3d_geom_msg.point.x = point_3d[0]
            point_3d_geom_msg.point.y = point_3d[1]
            point_3d_geom_msg.point.z = point_3d[2]

            # transform the point to the pointcloud frame using tf
            point_cloud_frame = self.camera_model.tfFrame()
            self.point_3d = self.listener.transformPoint(point_cloud_frame, point_3d_geom_msg)
            self.camera_cube_locator_marker_gen()

            # if min_depth > 600:
            #     point_3d = np.array(ray) * (min_depth - .2) / 1000
            #     point_3d_geom_msg = PointStamped()
            #     point_3d_geom_msg.header = msg.header
            #     point_3d_geom_msg.header.stamp = rospy.Time(0)
            #     point_3d_geom_msg.point.x = point_3d[0]
            #     point_3d_geom_msg.point.y = point_3d[1]
            #     point_3d_geom_msg.point.z = point_3d[2]
            #     point_mocap = self.listener.transformPoint('locobot_2', point_3d_geom_msg)
            #     print(point_3d)
            #     self.locobot.base.move_to_pose(point_mocap.point.x, point_mocap.point.y, 0, True)
            #     rospy.sleep(10)

    def info_callback(self, info_msg):
        # create a camera model from the camera info
        self.camera_model.fromCameraInfo(info_msg)

    def camera_cube_locator_marker_gen(self):
        #this is very simple because we are just putting the point P in the base_link frame (it is static in this frame)
        marker = Marker()
        marker.header.frame_id = self.point_3d.header.frame_id #"locobot/camera_depth_link"
        marker.header.stamp = rospy.Time.now()
        marker.id = 0
        marker.type = Marker.SPHERE
        # Set the marker scale
        marker.scale.x = 0.05  # radius of the sphere
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        # Set the marker pose
        marker.pose.position.x = self.point_3d.point.x
        marker.pose.position.y = self.point_3d.point.y
        marker.pose.position.z = self.point_3d.point.z
        # Set the marker color
        marker.color.a = 1.0 #transparency
        marker.color.r = 1.0 #red
        marker.color.g = 0.0
        marker.color.b = 0.0

        # Publish the marker
        self.camera_cube_locator_marker.publish(marker)

def main():
    rospy.init_node('cube_detection')
    detector = CubeDetector()
    rospy.spin()


if __name__ == "__main__":
    main()

