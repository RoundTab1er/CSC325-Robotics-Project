#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
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
                 pub_topic="object_core_image",
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
            # Call listener with CV data
            cv_img = self.image_listener(cv_img)

            # Republish to ROS
            ros_img = self.convert_cvimage(cv_img)
            self.image_pub.publish(ros_img)

    def convert_cvimage(self, cv_image):
        """Converts OpenCV image to ROS"""
        try:
            return self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        except CvBridgeError as e:
            print("ERROR: Could not convert open_cv image. ", e)

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
        # crap
        image_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.clbk)
        self.centroid = None
        self.bridge = CvBridge()


    def detect_objects(self, cv_image):
        """
        ImageStreamConverter listener callback.
        Processes CV image, looking for various objects.
        """

        # process shapes here...
        squares = self.process_squares(cv_image)
        #circles = self.process_circles(cv_image)

        # process detected shapes' colors here...

        # size detect here...

        # publish custom topic - bounding rectangle
        pub_shapes = { "square": squares, "circle": circles } # + triangles!
        #rospy.Publisher("shapes", AType)

        return cv_image

    def process_squares(self, cv_image):
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
                if w > min_size and h > min_size:
                    cv2.rectangle(cv_image, (x,y),(x+w,y+h), (200,0,40), 2)
                    detected_objs.append((x, y, w, h))
                    # crap
                    self.centroid = (x, y)

        return detected_objs


    def clbk(self, depth_img):
        # crap
        depth_image = self.bridge.imgmsg_to_cv2(depth_img, '32FC1')
        depth_array = np.array(depth_image, dtype=np.float32)
        print("centroid: ", self.centroid[0], self.centroid[1])
        print(depth_array[self.centroid[0], self.centroid[1]] / 1000.0)


    def process_circles(self, cv_image):
        cv_img_grey = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)

        cv_img_grey = cv2.medianBlur(cv_img_grey,5)

        circles = cv2.HoughCircles(cv_img_grey, cv2.HOUGH_GRADIENT, 1, 20,
                                   param1=200,
                                   param2=50,
                                   minRadius=20,
                                   maxRadius=200)

        for i in circles[0,:]:
            cv2.circle(cv_image,(i[0],i[1]),i[2],(0,255,0),2)
            cv2.circle(cv_image,(i[0],i[1]),2,(0,0,255),3)

def main(args):
    detector = ObjectDetector()
    rospy.init_node('object_detect', anonymous=True)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
