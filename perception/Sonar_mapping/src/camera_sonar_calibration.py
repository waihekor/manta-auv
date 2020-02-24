#!/usr/bin/env python
import os
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
import calibration_misc
import mapping_functions
import cv2


class MANUAL_CALIBRATION:




    def calibrationCallback(self,data):
        # Get input from user
        calibration_misc.print_userface()
        try:
            user_input = -1
            user_input = input()
            output = calibration_misc.input_switcher(user_input)
        except:
            output = -1
            print("Could not read input, it should be a single digit from the list\n")

        # A switch case for what to do with user input
        if output == 'Save':
            try:
                current_width, current_bearing, current_range = calibration_misc.Save_current_frame(self.sonar_data, self.image_data)
            # Appends current scan to list used for calibrating
                if current_width != -1 and current_bearing != -1 and current_range != -1:
                    self.pixel_width.append(current_width)
                    self.sonar_bearing.append(current_bearing)
                    self.sonar_range.append(current_range)
                    print("Current frame saved")
                else:
                    print("Not able to save current frame")
            except:
                print('Is not able to find video stream and/or sonar stream, check that both are running. \
                    If they are check if the correct topics are specified.')

        elif output == 'Calibrate':
            if len(self.pixel_width) > 0:
                self.calibration = 1
                self.a, self.b = calibration_misc.calibrate_line(self.pixel_width,self.sonar_bearing,self.sonar_range)
                print("Calibration parameters written to calibration.txt\n")
            else: 
                print("You have to save some frames before calibraiting")

        elif output == 'CalibrateCurve':
            if len(self.pixel_width) > 0:
                self.calibration = 2
                self.a, self.b, self.c = calibration_misc.fit_curve(self.pixel_width,self.sonar_bearing)
            else:
                print("You must save som frames before calibraiting")

        elif output == 'Calibrate3deg':
            if len(self.pixel_width) > 0:
                self.calibration = 3
                self.a, self.b, self.c, self.d = calibration_misc.fit_3deg(self.pixel_width,self.sonar_bearing)
            else:
                print('You must save some frames before calibraitings')

        elif output == 'Test':
            try:
                calibration_misc.test_calibration(self.sonar_data, self.image_data, self.a, self.b, self.c, self.d, self.calibration)
            except:
                print('Is not able to find video stream and/or sonar stream, check that both are running. \
                    If they are check if the correct topics are specified.')
      
        elif output == 'Close':
            print("Closing application...")
            rospy.signal_shutdown("Shutting down")
   
    def visualisingCallback(self,data):
        # Visualising both sonar and camera feed
        calibration_misc.convert_to_openCV(self.image_data)
        mapping_functions.imageCurrentScan(self.sonar_data,WALL_WIDTH=3,SCALE=100)

    def imageCallback(self,data):
        self.image_data = data

    def sonarCallback(self,data):
        self.sonar_data = data

    def __init__(self):        
        # Necesarry variables
        self.sonar_data = LaserScan()
        self.image_data = Image()
        self.image = 0
        self.pixel_width = []
        self.sonar_bearing = []
        self.sonar_range = []
        self.a = 0
        self.b = 0
        self.c = 0
        self.d = 0
        self.calibration = 0
        # Necesarry callback functions
        rospy.Subscriber('manta/sonar',LaserScan,self.sonarCallback)
        rospy.Subscriber('manta/manta/camerafront/camera_image',Image,self.imageCallback)
        #rospy.Timer(rospy.Duration(1.0/10.0),self.visualisingCallback)
        rospy.Timer(rospy.Duration(1.0),self.calibrationCallback)
        

if __name__ == "__main__":
    rospy.init_node('camera_sonar_calibration',anonymous=True,disable_signals=True)
    MANUAL_CALIBRATION()
    rospy.spin()



