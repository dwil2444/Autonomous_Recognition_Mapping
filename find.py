#!/usr/bin/env python

import sys
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import MultiArrayDimension

class image_converter:

    def __init__(self):
        self.msg = Int32MultiArray()
        # self.msg=CompressedImage()
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw/compressed", CompressedImage, self.imageCallback)
        self.lane_pub  = rospy.Publisher("/lane", Int32MultiArray, queue_size = 10)
        # self.lane_pub  = rospy.Publisher("/lane", CompressedImage, queue_size = 1)

    def imageCallback(self, data):
        # convert compressed image to cv2 image
        np_arr = np.fromstring(data.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # get the image dimension
        (rows,cols,channels) = cv_image.shape

        # convert image to black and white by thresholding
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        thresh     = 230
        bw_image   = cv2.threshold(gray_image, thresh, 255, cv2.THRESH_BINARY)[1]

        # image processing for denoising the image
        kernel = np.ones((5,5), np.uint8)

        # # Method I : erosion & dilation
        # erosion = cv2.erode(bw_image, kernel, iterations = 2)
        # dilation = cv2.dilate(erosion, kernel, iterations = 2)
        
        # # Method II : Open (open is similar to dilating after eroding)
        opening = cv2.morphologyEx(bw_image, cv2.MORPH_OPEN, kernel)

        # Crop the image to show the lower half of the image
        lower_half = opening[rows/2:rows, 0:cols]
        # lower_half = opening[5*(rows/8):7*(rows/8), 0:cols]
        
        # print(lower_half)
        # visualization for tuning (For better performance, you can comment this part out during competition)
        # cv2.imshow("Image window", lower_half)
        cv2.waitKey(1)

        # get the dimension for the ouput
        (H,W) = lower_half.shape

        # flatten the image
        listdata = lower_half.flatten()

        self.msg.layout.dim.append(MultiArrayDimension())
        self.msg.layout.dim.append(MultiArrayDimension())
        self.msg.layout.dim[0].size = H
        self.msg.layout.dim[1].size = W
        self.msg.data = listdata
        self.lane_pub.publish(self.msg)

        # self.msg.header.stamp=rospy.Time.now()
        # self.msg.format='jpeg'
        # self.msg.data=listdata
        # self.lane_pub.publish(self.msg)
        
        
def main():
    ic = image_converter()
    rospy.init_node('findLane', anonymous=True)
    try:
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
        # rate = rospy.Rate(10)
        # while not rospy.is_shutdown():

    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass
