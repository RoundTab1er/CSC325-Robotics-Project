#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class DepthHandler:

    def __init__(self):
        self.image_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.img_callback)
        #self.shape_sub = rospy.Subscriber("shape", aTYPE, self.img_callback)
        self.bridge = CvBridge()

    def img_callback(self, depth_img):
        depth_image = self.bridge.imgmsg_to_cv2(depth_img, '32FC1')
        depth_array = np.array(depth_image, dtype=np.float32)
        print(depth_array[40, 60] / 1000.0)


def main(args):
    detector = DepthHandler()
    rospy.init_node('map_handler', anonymous=True)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
