#!/usr/bin/env python

import rospy
from vortex_msgs.msg import ObjectPlacement
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import object_placement_misc


# Må lese inn width and height på kart, i tillegg til variablene a,b,c,d for fitting 
# Globale variabler
WIDTH = 500
HEIGHT = 500
SCALE = 10
a = -9.50006051716e-09
b = 5.89861616279e-06
c = -0.0024068634106
d = 0.861113632726



class OBJECT_PLACEMENT:

    # Checks if YOLOv3 is able to detect an object, AND returns the width pixel of the object. 
    def placeObjectCallback(self,data):
        try:
            [statement, pixel_width] = object_placement_misc.checkForObject(self.image) # Placeholder function will subscribe to topic in final code
        
        except:
            statement = False
            print("Can't find image stream")
        if statement:
            bearing = object_placement_misc.find_sonar_pos(pixel_width,a,b,c,d)
            depth = object_placement_misc.find_sonar_depth(sonar_data,bearing)
            if depth != False:
                obj_pos_msg = object_placement_misc.find_object_position(bearing,depth,self.ekf_data,WIDTH,HEIGHT,SCALE)
                self.pub.publish(obj_pos_msg)


    def imageCallback(self,data):
        self.image = data

    def sonarCallback(self,data):
        print("Hello")
        self.sonar_data = data
  
    def ekfCallback(self,data):
        self.ekf_data = data

    def __init__(self):

        # Variables
        self.ekf_data
        self.sonar_data
        self.image

        # Publshers
        self.pub = rospy.Publisher('/manta/object_position',ObjectPlacement,queue_size=10)

        # Subscribers
        rospy.Subscriber('manta/sonar',LaserScan,self.sonarCallback)
        rospy.Subscriber('/odometry/filtered',Odometry,self.ekfCallback)
        rospy.Subscriber('manta/manta/camerafront/camera_image',Image,self.imageCallback)
        rospy.Subscriber('manta')
        rospy.Timer(1/10,self.placeObjectCallback)




if __name__ = '__main__':
    rospy.init_node('place_object_in_map',anonymous=True)
    OBJECT_PLACEMENT()
    rospy.spin()














####### Demo Object While waiting for Object detection #################