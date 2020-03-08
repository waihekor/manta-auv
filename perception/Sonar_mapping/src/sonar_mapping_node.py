#!/usr/bin/env python
import rospy
import cv2 as cv
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import mapping_functions
import occupancy_gird_publisher
from nav_msgs.msg import OccupancyGrid
from vortex_msgs.msg import ObjectPlacement

############ Constant variables ###########################################
# Visialusation
GLOBAL_MAP = rospy.get_param("/GLOBAL_MAP",False)
LOCAL_MAP = rospy.get_param("/LOCAL_MAP",False)
PUBLISH_GLOBAL_MAP = rospy.get_param("/PUBLISH_GLOBAL_MAP",True)
WALL_WIDTH = rospy.get_param("/WALL_WIDTH",1)
SCALE = rospy.get_param("/SCALE",5)
MAP_SIZE = rospy.get_param("MAP_SIZE",500)
###########################################################################


class ROS_MAPPING:
    # Constructor
    def __init__(self):
         # If a map has to be published an msg must be initiated and a publisher has to be declared
        if PUBLISH_GLOBAL_MAP == True:
            self.map_msg = occupancy_gird_publisher.initiateMapMsg()
            self.pub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)
            self.map_pub = np.empty((MAP_SIZE,MAP_SIZE), np.float)
            self.map_pub.fill(-0.01)
        
        # Necesarry variables
        self.sonar_data = LaserScan()
        self.ekf_data = Odometry()
        self.object_position_data = ObjectPlacement()
        self.map = np.ones((MAP_SIZE,MAP_SIZE),np.float32)
        print(SCALE)
        
        # Necesarry callback functions
        rospy.Subscriber('manta/sonar',LaserScan,self.sonarCallback)
        rospy.Subscriber('/odometry/filtered',Odometry,self.ekfCallback)
        rospy.Subscriber('/manta/object_position',ObjectPlacement,self.objectPositionCallback)
        rospy.Timer(rospy.Duration(1.0/10.0),self.mappingCallback)

    # Class functions
    def createImageGlobalMap(self):
        mapping_functions.imageGlobalMap(self.sonar_data,self.ekf_data,self.map,WALL_WIDTH,SCALE, self.object_position_data)

    def createImageCurrentScan(self):
        mapping_functions.imageCurrentScan(self.sonar_data,WALL_WIDTH,SCALE,self.object_position_data)

    def publishGlobalMap(self):
        self.map_pub, map_msg = occupancy_gird_publisher.publishGlobalMap(self.sonar_data,self.ekf_data,self.map_pub,WALL_WIDTH,SCALE,self.map_msg)
        self.pub.publish(self.map_msg)
        
    def sonarCallback(self,data):
        self.sonar_data = data
  
    def ekfCallback(self,data):
        self.ekf_data = data

    def mappingCallback(self,data): 
        if GLOBAL_MAP == True:
            self.createImageGlobalMap()
        if LOCAL_MAP == True:
            self.createImageCurrentScan()   
        if PUBLISH_GLOBAL_MAP == True:
            self.publishGlobalMap()

    def objectPositionCallback(self,data):
        self.object_position_data = data
  


        

#### INIT of node ##################################
if __name__ == '__main__':
    rospy.init_node('sonar_mapping',anonymous=True)
    ROS_MAPPING()
    rospy.spin()
####################################################

