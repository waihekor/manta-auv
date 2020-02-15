#!/usr/bin/env python
import cv2 as cv
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import misc



def imageCurrentScan(data,WALL_WIDTH,SCALE):
    # Array containing current scan data
        sonarDisplayArray = np.ones((1500,1000), np.float32)
        positionManta = [1498,499]
        angle_increment = data.angle_increment
        current_angle = data.angle_min
        scan_ranges = data.ranges
        for index, range in enumerate(scan_ranges):
            if range < 44440:
                # Projecting point from the sonar onto a 2D-plane given range and angle of current ping
                scan_width = math.sin(current_angle)*range
                scan_height = math.sqrt(pow(range,2) - pow(scan_width,2))
                array_height = positionManta[0] - int(scan_height*SCALE)
                array_width = positionManta[1] - (scan_width*SCALE)

                # Colors squares between surface and solid gray to represent area without obstacles and black to represent solid walls
                rr, cc, val = misc.getLineBetweenPoints(array_height,array_width,positionManta[0],positionManta[1])
                sonarDisplayArray[rr,cc] = 0.8
                sonarDisplayArray[int(array_height-WALL_WIDTH):int(array_height+WALL_WIDTH),int(array_width-WALL_WIDTH):int(array_width+WALL_WIDTH)] = 0
                sonarDisplayArray[(positionManta[0]-20):(positionManta[0]),(positionManta[1]-10):(positionManta[1]+20)] = 0

            # Each scan containts X-amount of pings with an increment in angle between the pings which have to be updated
            current_angle+=angle_increment 
        
        #Visualising sonar image
        cv.imshow("sonarDisplay",sonarDisplayArray)
        cv.waitKey(1)
            

def imageGlobalMap(sonar_data,ekf_data,map,WALL_WIDTH,SCALE):
    # Finding Mantas current position and yaw from the EKF by transforming Quaternions to Euler
    ekf_x = ekf_data.pose.pose.position.x
    ekf_y = ekf_data.pose.pose.position.y
    map_height, map_width = map.shape
    positionManta = [int((map_height/2)-ekf_x*SCALE),int((map_width/2)-ekf_y*SCALE)]
    quat_yaw = ekf_data.pose.pose.orientation.z
    quat_pitch = ekf_data.pose.pose.orientation.y
    quat_roll = ekf_data.pose.pose.orientation.x
    quat_real = ekf_data.pose.pose.orientation.w
    [ekf_yaw,ekf_pitch,ekf_roll] = misc.quaternion_to_euler(quat_roll, quat_pitch, quat_yaw, quat_real)
    #map=np.ones((5000,5000),np.float32)
    # Extracting sonar scan data
    angle_increment = sonar_data.angle_increment
    current_angle = sonar_data.angle_min
    scan_ranges = sonar_data.ranges
  

    # Looping through the scans drawing the current scan on the exsisting map
    for index, depth in enumerate(scan_ranges):
        if depth < 20:
            # Projecting point from the sonar onto a 2D-plane given range and angle of current ping
            scan_width = math.sin(current_angle)*depth
            scan_height = math.sqrt(pow(depth,2) - pow(scan_width,2))                                          #math.sqrt(pow(range,2) - pow(scan_width,2))
            array_height = positionManta[0] - int(scan_height*SCALE)
            array_width = positionManta[1] - (scan_width*SCALE)

            # Transforms ping from sonar_frame to auv_frame
            point=np.array([[array_height], [array_width]])
            center=np.array([[positionManta[0]], [positionManta[1]]])
            array_height,array_width = misc.rotatePointAroundCenter(point,center,ekf_yaw)

            # Colors squares between surface and solid gray to represent area without obstacles 
            rr, cc, val = misc.getLineBetweenPoints(array_height,array_width,positionManta[0],positionManta[1])
            for j in range(len(rr)):
                if map[rr[j],cc[j]] != 0:
                    map[rr[j],cc[j]] = 0.8

            # Colors obstacles detected to black
            map[int(array_height-WALL_WIDTH):int(array_height+WALL_WIDTH),int(array_width-WALL_WIDTH):int(array_width+WALL_WIDTH)] = 0
            

        # Each scan containts X-amount of pings with an increment in angle between the pings which have to be updated
        current_angle+=angle_increment 
    

    map[(positionManta[0]-2):(positionManta[0]),(positionManta[1]-1):(positionManta[1]+2)] = 0
    #Visualising sonar image
    cv.namedWindow('MAP_DISPLAY',cv.WINDOW_NORMAL)
    cv.resizeWindow('MAP_DISPLAY', 1500,1500)
    cv.imshow("MAP_DISPLAY",map)
    cv.waitKey(1)
    map[(positionManta[0]-2):(positionManta[0]),(positionManta[1]-1):(positionManta[1]+2)] = 0.8
    return map

    # See sonar FOV
    '''
    range = 10
    scan_width = math.sin(-0.785)*range
    scan_height = math.sqrt(pow(range,2) - pow(scan_width,2))                                          #math.sqrt(pow(range,2) - pow(scan_width,2))
    array_height = positionManta[0] - int(scan_height*SCALE)
    array_width = positionManta[1] - (scan_width*SCALE)
    point=np.array([[array_height], [array_width]])
    center=np.array([[positionManta[0]], [positionManta[1]]]) 
    array_height,array_width = misc.rotatePointAroundCenter(point,center,ekf_yaw)  
    rr, cc, val = misc.getLineBetweenPoints(array_height,array_width,positionManta[0],positionManta[1])
    map[rr,cc] = 0
    
    scan_width = math.sin(0.785)*range
    scan_height = math.sqrt(pow(range,2) - pow(scan_width,2))                                          #math.sqrt(pow(range,2) - pow(scan_width,2))
    array_height = positionManta[0] - int(scan_height*SCALE)
    array_width = positionManta[1] - (scan_width*SCALE)
    point=np.array([[array_height], [array_width]])
    center=np.array([[positionManta[0]], [positionManta[1]]]) 
    array_height,array_width = misc.rotatePointAroundCenter(point,center,ekf_yaw)  
    rr, cc, val = misc.getLineBetweenPoints(array_height,array_width,positionManta[0],positionManta[1])
    map[rr,cc] = 0
    '''
