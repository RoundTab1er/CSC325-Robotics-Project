#!/usr/bin/env python
from __future__ import print_function
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import numpy as np


class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("camera/rgb/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as err:
      print(err)

    ######

    gray = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY )

    ret,thresh = cv2.threshold(gray,127,255,1)
    _,contours,h = cv2.findContours(thresh,1,2)

    for cnt in contours:
      approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
      if len(approx)==4:
        #print("square")
        (x, y, w, h) = cv2.boundingRect(approx)
        if w > 10 and h > 10:
            cv2.rectangle(cv_image, (x,y),(x+w,y+h), (200,0,40), 2)
        #cv2.drawContours(cv_image,[cnt],0,100,-1)

    #cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
