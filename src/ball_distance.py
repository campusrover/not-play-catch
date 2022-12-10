#!/usr/bin/env python3

"""
Calculate the distance of the ball from the world origin and publish the image of the ball with a box around it 
as well as publish the location of the ball to be published as a transform
"""

import rospy 
import imutils
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import Point, Quaternion, Pose

class Ball():

    def __init__(self, diameter : float):
        # Subscribers
        self.cam_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.cv_callback)
        self.gray_mask_sub = rospy.Subscriber("/catch/grayed/compressed/", CompressedImage, self.gray_cb)

        # Publishers
        self.box_pub = rospy.Publisher("/catch/box/compressed", CompressedImage, queue_size=10)
        self.pose_pub = rospy.Publisher("/catch/ball_pose", Pose, queue_size=10)

        # in cm - determined through measurements
        self.ball_diameter = diameter 
        # calculated using focal_length = (distance * pixel_size) / width - ball diameter in this case 
        self.focal_length = 624 
        # in cm - determined through measurements
        self.area_width = 57.1 
        self.area_height = 47 
        # in m - determined through measurements - offset of the origin of the world
        self.x_displacement = 0.14
        self.y_displacement = 0.27
        # in cm - determined through placement and measurements so that it can see the whole board
        self.camera_distance_from_ground = 40 

    # callback for the image 
    def cv_callback(self, msg):
        self.rgb_image = CvBridge().imgmsg_to_cv2(msg)
        # get the image size
        self.img_width = self.rgb_image.shape[1]
        self.img_height = self.rgb_image.shape[0]
        # ratio between pixels and the real world so that we can map pixels to coordinates
        self.area_width_pixel_ratio = self.area_width / self.img_width 
        self.area_height_pixel_ratio = self.area_height / self.img_height 

    # callback for the grey masked image and then run create counters
    def gray_cb(self, msg):
        self.grey_masked_image = CvBridge().compressed_imgmsg_to_cv2(msg)
        self.create_contours()

    # find the contours of the ball in the image
    def create_contours(self):
        # Lets try countours
        contours = cv.findContours(self.grey_masked_image, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)
        # find the markers of the contoured figure - hopefully it is the ball 
        marker = self.find_markers(contours)
        # if the marker is found, then print the box around the ball and publish
        if marker:
            self.print_box(marker)
        else: 
            self.pose_pub.publish(Pose(Point(0,0,0), Quaternion(0,0,0,1)))

    # publishes the image of the box around the ball and coordinates at the bottom of the screen
    def print_box(self, marker):
        # make a copy of the image 
        self.box_image = self.rgb_image.copy()
        M = cv.moments(self.grey_masked_image)
        # get the coordinates of the box
        coordinates = self.coordinates(marker)
        box = cv.boxPoints(marker) if imutils.is_cv2() else cv.boxPoints(marker)
        box = np.int0(box)

        if box[1][0] - box[0][0] >= 20:
            # draw the box around the object
            cv.drawContours(self.box_image, [box], -1, (0, 255, 0), 2)
            # put the text in the corner
            cv.putText(self.box_image, "%.2f, %.2f cm" % (coordinates[1] - self.x_displacement*100, coordinates[0] - self.y_displacement*100),
                (self.box_image.shape[1] - 300, self.box_image.shape[0] - 20), cv.FONT_HERSHEY_SIMPLEX,
                1.0, (0, 255, 0), 3)
            # create the proper msg type and publish it
            box_msg = CvBridge().cv2_to_compressed_imgmsg(self.box_image)
            self.box_pub.publish(box_msg)
            self.pose_pub.publish(Pose(Point(coordinates[1]/100-self.x_displacement, -(coordinates[0]/100-self.y_displacement), 0.01), Quaternion(0,0,0,1)))

    # compute the coordinates of the physical plane from the pixel plane
    def coordinates(self, marker):
        return ((marker[0][1] + marker[1][1]/2)*self.area_height_pixel_ratio, (marker[0][0] + marker[1][0]/2)*self.area_width_pixel_ratio)
    
    # find the contours from the image and return the largest area which is hopefully the ball 
    def find_markers(self, contours):
        cnts = imutils.grab_contours(contours)
        if cnts:
            c = max(cnts, key = cv.contourArea)
            c_list = cv.minAreaRect(c)
            return c_list
        else:
            return None

if __name__ == "__main__":
    rospy.init_node("ball_distance")
    try:
        Ball(diameter=2.5)
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
    
