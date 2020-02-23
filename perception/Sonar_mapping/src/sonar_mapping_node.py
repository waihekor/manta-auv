#!/usr/bin/env python
import rospy
import cv2 as cv
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import misc
import mapping_functions
import occupancy_gird_publisher
from nav_msgs.msg import OccupancyGrid

############ Global variables, into YAML file in the future ###############
# Visialusation
GLOBAL_MAP =False
LOCAL_MAP = False 
PUBLISH_GLOBAL_MAP = True
PUBLISH_LOCAL_MAP = False
WALL_WIDTH = 1
SCALE = 5
###########################################################################




class ROS_MAPPING:

    #### Class attribtues - Constant variables - Read from YAML #####



    #################################################################

    #### Class handeling functions ##########################

    def createImageGlobalMap(self):
        mapping_functions.imageGlobalMap(self.sonar_data,self.ekf_data,self.map,WALL_WIDTH,SCALE)

    def createImageCurrentScan(self):
        mapping_functions.imageCurrentScan(self.sonar_data,WALL_WIDTH,SCALE)

    def publishGlobalMap(self):
        self.map, map_msg = occupancy_gird_publisher.publishGlobalMap(self.sonar_data,self.ekf_data,self.map,WALL_WIDTH,SCALE,self.map_msg)
        self.pub.publish(self.map_msg)

    def publishLocalMap(self):
        pass
        
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
        if PUBLISH_LOCAL_MAP == True:
            self.publishLocalMap()

    def __init__(self):
         # If a map has to be published an msg must be initiated and a publisher has to be declared
        if PUBLISH_GLOBAL_MAP == True:
            self.map_msg = occupancy_gird_publisher.initiateMapMsg()
            self.pub = rospy.Publisher('manta/global_map', OccupancyGrid, queue_size=10)
        
        if PUBLISH_LOCAL_MAP == True:
            pass
        
        # Necesarry variables
        self.sonar_data = LaserScan()
        self.ekf_data = Odometry()
        self.map = np.ones((500,500),np.float32)
        
        # Necesarry callback functions
        rospy.Subscriber('manta/sonar',LaserScan,self.sonarCallback)
        rospy.Subscriber('/odometry/filtered',Odometry,self.ekfCallback)
        rospy.Timer(rospy.Duration(1.0/10.0),self.mappingCallback)
        

#### INIT of node ##################################
if __name__ == '__main__':
    rospy.init_node('sonar_mapping',anonymous=True)
    ROS_MAPPING()
    rospy.spin()
###################################################

