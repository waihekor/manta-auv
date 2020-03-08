#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from vortex_msgs.msg import ObjectPlacement
from darknet_ros_msgs.msg import BoundingBoxes
import object_placement_misc
import object_class

################################################
# Globale variabler
MAP_SIZE = rospy.get_param("MAP_SIZE",500)
SCALE = rospy.get_param("/SCALE",5)
a = rospy.get_param("/a",-100)
b = rospy.get_param("/b",-100)
c = rospy.get_param("/c",-100)
d = rospy.get_param("/d",-100)
################################################


class OBJECT_PLACING:
    # Constructor
    def __init__(self):

        # Variables 
        self.bbox = BoundingBoxes()
        self.sonar_data = LaserScan()
        self.image = Image()
        self.ekf_data = Odometry()
        self.bootlegger = object_class.LOCATION_OF_OBJECT("Bootlegger")
        self.gman = object_class.LOCATION_OF_OBJECT("G-man")
        self.tommygun = object_class.LOCATION_OF_OBJECT("Tommygun")
        self.badge= object_class.LOCATION_OF_OBJECT("Badge")
        # Publshers
        self.pub = rospy.Publisher('/manta/object_position',ObjectPlacement,queue_size=10)

        # Subscribers
        rospy.Subscriber('manta/sonar',LaserScan,self.sonarCallback)
        rospy.Subscriber('/odometry/filtered',Odometry,self.ekfCallback)
        rospy.Subscriber('/darknet_ros/bounding_boxes',BoundingBoxes,self.objectCallback)
        rospy.Timer(rospy.Duration(1.0),self.publishCallback)
        # For testing purposes
        #rospy.Subscriber('/manta/manta/camerafront/camera_image',Image,self.imageCallback)
        #rospy.Timer(rospy.Duration(1.0/10.0),self.placeObjectCallback)

    # Class functions
    def objectCallback(self,data):
        self.bbox = data
        list_of_objects = object_placement_misc.find_pixel_placement(self.bbox)
        for objects in list_of_objects:
            bearings = object_placement_misc.calculate_bearing(objects,a,b,c,d)
            depth = object_placement_misc.calculate_depth(bearings,self.sonar_data)
            map_coordinates = object_placement_misc.calculate_map_coordinates(self.ekf_data, self.sonar_data, bearings, depth, MAP_SIZE, MAP_SIZE,SCALE)
            if objects[0] == "bootlegger":
                self.bootlegger.update(map_coordinates)
            elif objects[0] == "g-man":
                self.gman.update(map_coordinates)
            elif objects[0] == "tommy-gun":
                self.tommygun.update(map_coordinates)
            elif objects[0] == "badge":
                self.badge.update(map_coordinates)

    def publishCallback(self,data):
        # Initializing msg
        msg = ObjectPlacement()
        # Filling msg
        msg.bootlegger_found = self.bootlegger.detected
        msg.bootlegger_object_x = self.bootlegger.x_pos
        msg.bootlegger_object_y = self.bootlegger.y_pos

        msg.gman_found = self.gman.detected
        msg.gman_object_x = self.gman.x_pos
        msg.gman_object_y = self.gman.y_pos

        msg.badge_found = self.badge.detected
        msg.badge_object_x = self.badge.x_pos
        msg.badge_object_y = self.badge.y_pos

        msg.tommygun_found = self.tommygun.detected
        msg.tommygun_object_x = self.tommygun.x_pos
        msg.tommygun_object_y = self.tommygun.y_pos

        # Publishing msg
        self.pub.publish(msg)

    def sonarCallback(self,data):
        self.sonar_data = data
  
    def ekfCallback(self,data):
        self.ekf_data = data

    ####### Demo functions while waiting for Object detection #################
    # Only necesarry when testing placeholder function, darknet_ros gives pixel values directly so no need to look at image
    def imageCallback(self,data):
        self.image = data

    '''
    # Checks if the algorithm is able to calculate a position for a red object, only necesarry when testing
    def placeObjectCallback(self,data):
        try:
            [statement, pixel_width] = object_placement_misc.checkForObject(self.image) # Placeholder function will subscribe to topic in final code
        except:
            statement = False
            print("Can't find image stream")
        #if statement:
        #    bearing = object_placement_misc.find_sonar_pos(pixel_width,a,b,c,d)
        #    depth = object_placement_misc.find_sonar_depth(self.sonar_data,bearing)
        #    if depth != False:
        #        obj_pos_msg = object_placement_misc.find_object_position(bearing,depth,self.ekf_data,WIDTH,HEIGHT,SCALE,self.sonar_data)
        #        obj_pos_msg = object_placement_misc.place_objects_in_map(bearing,depth,self.ekf_data,WIDTH,HEIGHT,SCALE,self.sonar_data, self.bbox)
        #        self.pub.publish(obj_pos_msg)
    '''

#### INIT of node ##################################
if __name__ == '__main__':
    rospy.init_node('place_object_in_map',anonymous=True)
    OBJECT_PLACING()
    rospy.spin()
####################################################













