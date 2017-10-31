#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class DepthHandler:

    def __init__(self):
        self.image_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_updated)
        self.shape_sub = rospy.Subscriber("/sticky_wickets/object_data", String, self.shape_updated)
        self.bridge = CvBridge()

        self.depth_image_data = None

    def depth_updated(self, depth_image):
        depth_image = self.bridge.imgmsg_to_cv2(depth_image, '32FC1')
        self.depth_image_data = np.array(depth_image, dtype=np.float32)

    def shape_updated(self, shapes_data):
        if self.depth_image_data is not None:
            shapes_dict = eval(shapes_data.data)
            for square in shapes_dict["square"]:
                print(self.depth_image_data[square[0], square[1]] / 1000.0)




def main(args):
    detector = DepthHandler()
    rospy.init_node('map_handler', anonymous=True)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
