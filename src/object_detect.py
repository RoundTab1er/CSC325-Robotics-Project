#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

class ImageStreamConverter:
    """
    Open CV and ROS image stream converter. Responsible for subscribing to
    camera topics and publishing new topic. The primary function is allowing
    intermediate OpenCV image manipulation, through the cv_image_listener
    initializer parameter.
    """
    def __init__(self,
                 cv_image_listener,
                 pub_topic="/sticky_wickets/object/image", # NOTE: This is a topic is only for testing
                 sub_topic="camera/rgb/image_raw"):
        self.image_sub = rospy.Subscriber(sub_topic, Image, self.img_callback)
        self.image_pub = rospy.Publisher(pub_topic, Image)

        self.bridge = CvBridge()
        self.image_listener = cv_image_listener

    def img_callback(self, ros_img):
        """
        ROS camera stream image callback. Converts ROS image to CV image,
        calls the listener, and republishes on new ROS topic.
        """
        cv_img = self.convert_rosimage(ros_img)

        if cv_img is not None:
            cv_img = self.image_listener(cv_img)
            ros_img = self.convert_cvimage(cv_img)

            if ros_img is not None:
                self.image_pub.publish(ros_img)

    def convert_cvimage(self, cv_image):
        """Converts OpenCV image to ROS"""
        try:
            return self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        except CvBridgeError as e:
            print("ERROR: Could not convert open_cv image. ", e)
            return None

    def convert_rosimage(self, ros_image):
        """Converts ROS image to OpenCV"""
        try:
            return self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError as e:
            print("ERROR: Could not convert ros image. ", e)
            return None


class ObjectDetector:
    """
    Basic object detector. Detects objects using basic shape and color
    attributes. This is fully decoupled from ROS, and solely relies
    on OpenCV for image analysis and detection. It obtains the OpenCV
    Image stream by initializing an ImageStreamConverter instance
    which converts ROS to OpenCV.
    """
    def __init__(self):
        self.cv_converter = ImageStreamConverter(self.detect_objects)
        self.object_pub = rospy.Publisher("/sticky_wickets/object_data", String)

    def detect_objects(self, cv_image):
        """
        ImageStreamConverter listener callback.
        Processes CV image, looking for various objects.
        """
        squares = self.process_squares(cv_image)
        #circles = self.process_circles(cv_image)

        squares = self.analyze_shape_colors(squares, cv_image)

        # Publish custom topic - bounding rectangles (string message type)
        pub_shapes = str({ "square": squares }) # "circle": circles, "tri": triangles!
        self.object_pub.publish(pub_shapes)

        return cv_image

    def process_squares(self, cv_image):
        """
        Detects rectangular objects in CV Image
        """
        cv_img_grey = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)

        ret, thresh = cv2.threshold(cv_img_grey, 127, 255, 1)
        _, contours, h = cv2.findContours(thresh, 1, 2)
        epsilon_approx = 0.11 # Be careful with this constant
        min_size = 18

        detected_objs = []

        for cnt in contours:
            approx = cv2.approxPolyDP(cnt,epsilon_approx*cv2.arcLength(cnt,True),True)
            if len(approx) == 4:
                (x, y, w, h) = cv2.boundingRect(approx)
                if w > min_size and h > min_size and w < 300:
                    cv2.rectangle(cv_image, (x,y),(x+w,y+h), (200,0,40), 2)
                    detected_objs.append((x, y, w, h))

        return detected_objs

    def process_circles(self, cv_image):
        """
        Detects circles objects in CV Image
        """
        cv_img_grey = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)
        cv_img_grey = cv2.medianBlur(cv_img_grey, 5)

        circles = cv2.HoughCircles(cv_img_grey, cv2.HOUGH_GRADIENT, 1, 20,
                                   param1=200,
                                   param2=50,
                                   minRadius=20,
                                   maxRadius=200)

        for i in circles[0,:]:
            cv2.circle(cv_image,(i[0],i[1]),i[2],(0,255,0),2)
            cv2.circle(cv_image,(i[0],i[1]),2,(0,0,255),3)

    def analyze_shape_colors(self, bounding_rects, cv_image):
        """
        Analyzes colors within the bounding box of given detected
        shapes. If any shape's colors are incorrect, it will remove
        them from the returned list of shapes.
        """
        # Unimplemented
        return bounding_rects

def main(args):
    detector = ObjectDetector()
    rospy.init_node('object_detect', anonymous=True)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
