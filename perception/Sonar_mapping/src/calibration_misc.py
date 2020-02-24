import rospy
import numpy as np
import cv2
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from scipy.optimize import curve_fit
import math

# Prints a terminal userface
def print_userface():
    print("Application")
    print("###################################################\n")
    print("0. Save current frame")
    print("1. Calibrate with least_square over saved frames")
    print("2. Calibrate with curve optimization over saved frames")
    print("3. Calibrate with a 3D optimization over saved frames")
    print("4. Test current calibration")
    print("5. Close Application")
    print("###################################################\n")
    print("Waiting for user input: ")

# Takes inn two numpy arrays y and x, and finds the best line fit for all points, writes intersection and slope to calibration.txt
def fit_line_least_squares(y,x):
        # x and y is numpy arrays
        meanX = float(np.sum(x))/float(len(x))
        meanY = float(np.sum(y))/float(len(y))
        # Line equation: y = ax + b
        a_upper = 0
        a_lower = 0
        for i in range(len(x)):
            a_upper += ((float(x[i])-meanX)*(float(y[i])-meanY))
            a_lower += pow((float(x[i])-meanX),2)
        a = a_upper/a_lower
        b = meanY - a*meanX
        # Write fitted line parameters to file
        f = open('calibration.txt','w')
        f.write('a: ' + str(a) + '\nb: ' + str(b) + '\n')
        f.close()
        return a,b

# Converts a CompressedImage to an openCV image
def convert_to_openCV(image_msg):
    bridge = CvBridge()
    try: 
        image = bridge.imgmsg_to_cv2(image_msg, "bgr8")
        cv2.imshow('Display',image)
        cv2.waitKey(1)
    except:
        return []

# Switcher for handling user input
def input_switcher(user_input):
    Switcher = {
        0: 'Save',
        1: 'Calibrate',
        2: 'CalibrateCurve',
        3: 'Calibrate3deg',
        4: 'Test',
        5: 'Close',
    }
    return Switcher.get(user_input,'skip')
    
# Return bearing, range and pixel width of current msg
def Save_current_frame(sonar_data, image_data):
    # Extracting necesarry data from msgs
    angle_increment = sonar_data.angle_increment
    current_angle = sonar_data.angle_min
    scan_ranges = sonar_data.ranges
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(image_data, "bgr8")
    # Appending all promising scans to a list as tuples [range_of_scan, bearing_from_manta]
    possible_scans = get_promising_sonar_scans(angle_increment,current_angle,scan_ranges)
    #print(possible_scans)
    # Calculating average range and bearing
    if possible_scans:
        sonar_range, sonar_bearing = calculate_best_sonar_data(possible_scans)
    else:
        sonar_bearing = -1
        sonar_range = -1
        #print("Nope")
    # Finds current pixel width of pole
    pixel_width = get_pixel_width(image)

    return pixel_width, sonar_bearing, sonar_range 
    
# Calibrate sonar range as a function of pixel_widths from center to one side
def calibrate_line(pixel_widths, sonar_bearings, sonar_ranges):
    return fit_line_least_squares(sonar_bearings,pixel_widths)
       
# Writing fitted variables to a .txt file
def write_to_file():
    pass

# Returns all depth scans betlow a certain threshold with bearing
def get_promising_sonar_scans(angle_increment,current_angle,scan_ranges):
    possible_scans = []
    found_obj = False
    last_depth = scan_ranges[0]
    counter = 0
    for depth in scan_ranges:
        
        if last_depth > (depth + 5):
            found_obj = True
        elif last_depth < (depth - 3):
            found_obj = False
            break
        if found_obj:
            possible_scans.append(tuple([depth,current_angle]))
        counter += 1
        last_depth = depth
        current_angle += angle_increment
    return possible_scans

# Filters a colored pole and detects its bounding box
def get_pixel_width(image):
    # Transforms the image to HSV which is a color space easier to filter 
    image_hsv=cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # In the HSV colorspace RED is represented both from 170-180 and 0-10
    # 0-10
    lower_red = np.array([0,50,50])
    upper_red = np.array([10,255,255])
    mask0 = cv2.inRange(image_hsv, lower_red, upper_red)
    # 170-180
    lower_red = np.array([170,50,50])
    upper_red = np.array([180,255,255])
    mask1 = cv2.inRange(image_hsv, lower_red, upper_red)

    # Combining upper and lower values
    mask = mask0 + mask1
    
    # Finds contours
    _,contours,_ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    area = 0
    bbox = -1  
    pixel_width = -1
    for contour in contours:
        if cv2.contourArea(contour) > area:
            area = cv2.contourArea(contour)
            bbox = cv2.boundingRect(contour)
       
    if bbox != -1:
        pixel_width = (bbox[0] + int(bbox[0]+bbox[2]))/2
        ''' 
        # Visualising bounding box for troubleshooting purposes
        cv2.rectangle(image,(bbox[0],bbox[1]),(bbox[0]+bbox[2],bbox[1]+bbox[3]),(0,255,0), 3)  
        cv2.imshow('Display',image)
        cv2.waitKey()
        '''
    return pixel_width
    
# Calculates average angle and depth ---> Angle is most important
def calculate_best_sonar_data(possible_scans):
    average_angle = 0
    average_depth = 0
    for scan in possible_scans:
        average_angle += scan[1]
        average_depth += scan[0]
    average_angle = average_angle/float(len(possible_scans))
    average_depth = average_depth/float(len(possible_scans))
    ''' Might be useful for troubleshooting
    print(average_depth)
    print(average_angle)
    '''
    print(average_angle)
    return average_depth, average_angle

# Takes the a and b values and tries to predict the bearing to an object given pixel value
def test_calibration(sonar_data,image_data,a,b,c,d,calibration):
    pixel_width, _, _ = Save_current_frame(sonar_data, image_data)
    if calibration == 1:
        current_bearing = float(pixel_width)*a + b
        print(current_bearing)
    elif calibration == 2:
        current_bearing = math.pow(float(pixel_width),2)*a + float(pixel_width)*b + c
        print(current_bearing)
    elif calibration == 3:
        current_bearing = math.pow(float(pixel_width),3)*a + math.pow(float(pixel_width),2)*b + float(pixel_width)*c + d
        print(current_bearing)

# Curve function
def curve_function(x,a,b,c):
    return pow(a*x,2) + b*x + c

# Fits a curve to points
def fit_curve(x,y):
    [a,b,c],_ = curve_fit(curve_function,x,y)
    f = open('calibration.txt','w')
    f.write('a: ' + str(a) + '\nb: ' + str(b) + '\nc:' + str(c))
    return a,b,c

# Fits 3d polynominal to points
def fit_3deg(x,y):
    a,b,c,d = np.polyfit(x,y,3) 
    f = open('calibration.txt','w')
    f.write('a: ' + str(a) + '\nb: ' + str(b) + '\nc:' + str(c) + '\nd:' + str(d))
    f.close()
    return a,b,c,d
    