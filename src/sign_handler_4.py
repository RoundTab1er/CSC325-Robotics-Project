#!/usr/bin/env python
import roslib
import sys
import math
import rospy
import cv2
import numpy as np
import tf
from move_base_msgs.msg import MoveBaseGoal
from geometry_msgs.msg import TransformStamped, Pose, Point, Quaternion
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import OccupancyGrid

IMAGE_WIDTH = 640
IMAGE_TARGET = 320
DIST_COL_FROM_TARGET = IMAGE_SIZE - IMAGE_TARGET

class SignHandler:

    def __init__(self):
        self.image_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_updated)
        self.shape_sub = rospy.Subscriber("/sticky_wickets/object_data", String, self.shape_updated)
        self.react_pub = rospy.Publisher("/sticky_wickets/sign/map", Image, queue_size=10)
        self.move_pub = rospy.Publisher("/sticky_wickets/sign/move_goal", MoveBaseGoal, queue_size=0)
        #self.map_grid_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        self.bridge = CvBridge()
        self.depth_image_data = None
        self.signs = []
        self.listener = tf.TransformListener()


    def depth_updated(self, depth_image):
        """
        ROS depth image callback that decodes the depth data.
        """
        depth_image = self.bridge.imgmsg_to_cv2(depth_image, '32FC1')
        self.depth_image_data = np.array(depth_image, dtype=np.float32)

    def shape_updated(self, shapes_data):
        """
        ROS updated shape callback. Analyzes centroids of shapes,
        determining distance based on decoded image depth data.
        """
        if self.depth_image_data is not None:
            shapes_dict = eval(shapes_data.data)
            self.handle_signs(shapes_dict)

    def handle_signs(self, shapes_dict):
        """
        Handles reaction to detected signs. If within a certain distance,
        react to the sign. This method is invoked from shape_updated callback.
        """
        react_dist = 0.7

        for num_sides, shapes in shapes_dict.items():
            for (x, y, w, h) in shapes:
                shape_object = (x, y, w, h)
                centroid = self.get_centroid(shape_object)

                dist_in_mtrs = self.depth_image_data[centroid[1], centroid[0]] / 1000.0

                if dist_in_mtrs < react_dist and dist_in_mtrs != 0.0:
                    if num_sides == "4": 
                        self.investigate_sign(centroid, dist_in_mtrs)

                        self.map_handler(self, centroid)
                    elif num_sides == "0": 
                        self.react_to_stopper()
                    elif num_sides == "3": 
                        self.react_to_hazard()

    def get_centroid(self, sign_object):
        """
        Calculates the centroid of a detected shapes from the bounding box published on /sticky_wickets/object_data
        """
        (x, y, w, h) = sign_object

        centroid_x = x + (w/2)
        centroid_y = y + (h/2)

        centroid = (centroid_x, centroid_y)

        return centroid

    def react_to_hazard(self):
        """
        Publish a reaction to current sign status = hazard ahead
        """
        desired_movement = (math.pi, 0) #180 degrees?

        self.pub_move_goal(desired_movement)

    def react_to_stopper(self):
        """
        Publish a reaction to current sign status = avoid area (stop, turn)
        """
        desired_movement = (math.pi, 0) #Not sure what 180 degree turn is, assuming its in rads, so pi = 1/2 revolution

        self.pub_move_goal(desired_movement)

    def pub_move_goal(self, target):
    	"""
        Publishes move commands to the /sticky_wickets/sign/move_goal topic
        """
        move_goal_msg = MoveBaseGoal()

        move_goal_msg.target_pose.header.frame_id = "base_link"
        move_goal_msg.target_pose.header.stamp = rospy.Time.now()

        move_goal_msg.target_pose.pose.position.x = target[1]
        move_goal_msg.target_pose.pose.orientation.z = target[0]

        rospy.loginfo(target)
        self.move_pub.publish(move_goal_msg)

    def investigate_sign(self, shape_centroid, dist_in_mtrs):
		"""
        Drives to directly in front of square signs in order to approximate their location (proof of concept).
        Functions called sequentially to hopefully avoid command failing
        """
        self.align_to_sign(shape_centroid)

     	if dist_in_mtrs > 0.2:
	        self.drive_towards_sign(dist_in_mtrs)

    def drive_towards_sign(self, dist_in_mtrs):
    	"""
        Sends drive forward command towards sign 
        """
       	row_speed = 0.2

        desired_movement = (0.0, row_speed)

        self.pub_move_goal(desired_movement)

    def align_to_sign(self, shape_centroid):
    	"""
        Sends spin required to center sign in image
        """
    	target_col = shape_centroid[0]
        col_dist = IMAGE_TARGET - target_col

        self.align_to_sign(shape_centroid)

        col_ang_speed = self.mapAngle(col_dist, DIST_COL_FROM_TARGET, -DIST_ROW_FROM_TARGET, math.pi/16, -math.pi/16)

        desired_movement = (col_ang_speed, 0.0)

        pub_move_goal(desired_movement)

    def mapAngle(self, centroid_x, max_px, min_px, max_ang, min_ang):
        """
        Proportional controller that maps pixels off target to rotation. 
        """
        fromRange = max_px - min_px
        trueVal = centroid_x - min_px
        proportion = trueVal/float(fromRange)
        toRange = max_ang - min_ang
        rawToVal = proportion * toRange
        toVal = rawToVal + min_ang

        return toVal

    def get_pose(self):
    	"""
        Returns current pose of robot in map frame
        """
         (curr_pos,_) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))

         return (curr_pos, _)

    def convert_pos_to_cell(self, position):
    	"""
        Returns occupancy grid space of pose
        """
        pos_grid_x = int((position[0] - self.map_meta.origin.position.x) / self.map_meta.resolution)
        pos_grid_y = int((position[1] - self.map_meta.origin.position.y) / self.map_meta.resolution)
        rospy.loginfo([pos_grid_x, pos_grid_y])

        return [pos_grid_x, pos_grid_y]

    def save_loc(self, grid_location):
        return self.signs.append(location_pose)

    def make_sign_map(self, map_data):
    	"""
        Generates white image of equal size to gmapping map and returns the image. 
        """
        blank_map = np.zeros((self.map_data.info.height, self.map_data.info.width, 3), np.uint8)

        #makes image white
        blank_map.fill(255)

        return blank_map

    def should_add_to_map(self, grid_location, shape):
        """
        Returns true or false based on whether a bounding box should be added
        to the map or not (e.g. checks if it's already there [QR?], etc.)
        """
        if location not in signs:
            save_loc(self.signs, location)
            integrate_into_map(self.signs, location, shape)

    def integrate_into_map(sign_map, grid_location):
        """
        Integrates shape onto map
        """
        (x, y) = (grid_location[0], grid_location[1])
        color = (255, 0, 0)

        sign_map[x, y] = color      # (B, G, R)

        return sign_map

    def map_handler(self, centroid, shape):
    	pass

def main(args):
    sign_handler = SignHandler()
    rospy.init_node('sign_handler', anonymous=True)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
