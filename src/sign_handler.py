#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
import numpy as np
from move_base_msgs.msg import MoveBaseGoal
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

IMAGE_SIZE = (640, 480)
IMAGE_TARGET = (320, 240)
DIST_ROW_FROM_TARGET = IMAGE_SIZE[0] - IMAGE_TARGET[0]
DIST_COL_FROM_TARGET = IMAGE_SIZE[1] - IMAGE_TARGET[1]

class SignHandler:

    def __init__(self):
        self.image_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_updated)
        self.shape_sub = rospy.Subscriber("/sticky_wickets/object_data", String, self.shape_updated)
        self.react_pub = rospy.Publisher("/sticky_wickets/sign/map", Image, queue_size=10)
        self.move_pub = rospy.Publisher("/sticky_wickets/sign/move_goal", MoveBaseGoal, queue_size=10)
        self.map_grid_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        self.bridge = CvBridge()
        self.depth_image_data = None
        self.signs = []

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

            # NOTE: for testing purposes
            for square in shapes_dict["4"]:
                # INTEGRATE THIS DATA INTO THE MAP IF IT IS VALID
                print(self.depth_image_data[square[0], square[1]] / 1000.0)

    def handle_signs(self, shapes_dict):
        """
        Handles reaction to polygon signs. If a certain distance is reached,
        react to the sign. This method is invoked from shape_updated callback.
        """
        react_dist = 0.6

        ## Get location of object, see if there are any stored landmarks
        # that are within range, if not, investigate    

        for num_sides, shapes in shapes_dict.items():
            for (x, y, w, h) in shapes:
                centroid = get_centroid(shape_object)

                (curr_pos,_) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))

                grid_loc = get_location(centroid)

                if grid_loc not in signs:
                    investigate(centroid)
                    self.save_loc()

                dist_in_mtrs = self.depth_image_data[centroid[0], centroid[1]] / 1000.0
               
                if dist_in_mtrs < react_dist:
                    if num_sides == "4":
                        self.pub_loc()
                    elif num_sides == "0":
                        self.react_to_stopper()
                    elif num_sides == "3":
                        self.react_to_hazard()

    def get_centroid(sign_object):
        (x, y, w, h) = bounding_box

        centroid_x = x + (w/2)
        centroid_y = y + (h/2)

        centroid = (centroid_x, centroid_y)

        return centroid

    def mapAngle(val, fromhi, fromlo, tohi, tolo):
        # return math.pi/8 if val > 0 else -math.pi/8
        fromRange = fromhi - fromlo
        trueVal = val-fromlo
        proportion = trueVal/float(fromRange)
        toRange = tohi - tolo
        rawToVal = proportion * toRange
        toVal = rawToVal + tolo
        return toVal

    def investigate_sign(shape_centroid):
        target_col = centroid[0]
        target_row = centroid[1]

        col_dist = IMAGE_TARGET[0] - target_col
        row_dist = IMAGE_TARGET[1] - target_row

        col_ang_speed = mapAngle(col_dist, DIST_ROW_FROM_TARGET, -DIST_ROW_FROM_TARGET, math.pi/2, -math.pi/2)

        #need to manage moving forward somewhere, maybe using depth, but could also use height like here
        if row_dist > 2:
            row_speed = 0.2

        print(col_ang_speed)

        desired_movement = (col_ang_speed, row_speed)

        pub_move_goal(desired_movement)

    def pub_move_goal(self, target):
        move_goal_msg = MoveBaseGoal()

        move_goal_msg.goal.target_pose.position.x = target[1]
        move_goal_msg.goal.target_pose.orientation.z = target[0]

        self.move_pub.Publish(move_goal)

    def react_to_hazard(self):
        """
        Publish a reaction to current sign status = hazard ahead
        """
        desired_movement = (math.pi, 0)

        pub_move_goal(desired_movement)

        ### COULD DO THIS, AND HANDLE IN MOVER, OR WE  ###
        ### CAN JUST PUBLISH DESIRED MOVEMENT TO MOVER ###
        #self.react_pub.publish("avoid")

    def react_to_stopper(self):
        """
        Publish a reaction to current sign status = avoid area (stop, turn)
        """
        desired_movement = (math.pi, 0) #Not sure what 180 degree turn is, assuming its in rads, so pi = 1/2 revolution

        pub_move_goal(desired_movement)

        ### COULD DO THIS, AND HANDLE IN MOVER, OR WE  ###
        ### CAN JUST PUBLISH DESIRED MOVEMENT TO MOVER ###
        #self.react_pub.publish("turn")

    def convert_pos_to_cell(self, position):
        pos_grid_x = int((position[0] - self.map_meta.origin.position.x) / self.map_meta.resolution)
        pos_grid_y = int((position[1] - self.map_meta.origin.position.y) / self.map_meta.resolution)
        print [pos_grid_x, pos_grid_y]
        
        return [pos_grid_x, pos_grid_y]

    def get_sign_location(self, position):
        #is the robot pose in m?
        depth_offset = self.depth_image_data[square[0], square[1]] / 1000.0
        location = convert_pos_to_cell + depth_offset

        return location

    def add_loc_to_img(sign_map, location):
        location_string = str(location)

        cv2.putText(sign_map, location_string, location, cv2.FONT_HERSHEY_SIMPLEX, 2, 255) #location is (x, y)

    def save_loc(self, location):
        return self.signs.append(location)

    #function should only be called if sign map doesn't exist
    def make_sign_map(map_data):
        #generates black image of equal size to gmapping map
        blank_map = np.zeros((map_data.info.height, map_data.info.width, 3), np.uint8)

        return blank_map

    def should_add_to_map(self, location, shape):
        """
        Returns true or false based on whether a bounding box should be added
        to the map or not (e.g. checks if it's already there [QR?], etc.)
        """
        
        if location not in signs:
            save_loc(self.signs, location)
            integrate_into_map(self.signs, location, shape)

    def integrate_into_map(sign_map, location, shape):
        """
        Integrates a validated shape onto an image / map thing
        """

        (x, y) = location
        color = (0, 0, 0)

        if shape == "square":
            color = (255, 0, 0)
        elif shape == "circle":
            color = (0, 255, 0)
        elif shape == "trianle":
            color = (0, 0, 255)

        sign_map[x, y] = color      # (B, G, R)

        return sign_map

def main(args):
    sign_handler = SignHandler()
    rospy.init_node('sign_handler', anonymous=True)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
