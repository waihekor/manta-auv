import math
import common
from vortex_msgs.msg import ObjectPlacement
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import mapping_functions

# General functions
def find_sonar_pos(pixel_width,a,b,c,d):
    if d != -100.0:
        return math.pow(float(pixel_width),3)*a + math.pow(float(pixel_width),2)*b + float(pixel_width)*c + d
    elif c != -100.0:
        return math.pow(float(pixel_width),2)*a + float(pixel_width)*b + c
    else:
        return float(pixel_width)*a + b

# Placing objects from darknet_ros
def find_pixel_placement(bbox):
    list_of_class_and_pixel_values = []
    for box in bbox.bounding_boxes:
        # Structure --> Class, xmin, xmax
        bounding_box = (box.Class, box.xmin, box.xmax, box.ymin, box.ymax)
        list_of_class_and_pixel_values.append(bounding_box)
        return list_of_class_and_pixel_values

def calculate_bearing(bounding_boxes, a, b, c, d):
    x_coordinates = (bounding_boxes[1],bounding_boxes[2])
    max_bearing = find_sonar_pos(x_coordinates[0],a,b,c,d)
    min_bearing = find_sonar_pos(x_coordinates[1],a,b,c,d)
    return (min_bearing,max_bearing)

def calculate_depth(bearings, sonar_data):
    angle_increment = sonar_data.angle_increment
    current_angle = sonar_data.angle_min
    depth = []
    closest_scan = float("inf")
    for scan in sonar_data.ranges:
        if current_angle > bearings[0] and current_angle < bearings[1]:
            depth.append(scan)
            if scan < closest_scan:
                closest_scan = scan
        current_angle += angle_increment
    depth = list(filter(lambda a: a < closest_scan + 0.2,depth))
    return sum(depth) / (len(depth))

def calculate_map_coordinates(ekf_data, sonar_data, bearings, depth, width, height, scale):
    # Getting mantas current position and heading
    robot_x = -int(ekf_data.pose.pose.position.x)*scale + int(height/2.0)
    robot_y = -int(ekf_data.pose.pose.position.y)*scale + int(width/2.0)
    manta_x = -(ekf_data.pose.pose.position.x)*scale + (height/2.0)
    manta_y = -(ekf_data.pose.pose.position.y)*scale + (width/2.0)
    quat_yaw = ekf_data.pose.pose.orientation.z
    quat_pitch = ekf_data.pose.pose.orientation.y
    quat_roll = ekf_data.pose.pose.orientation.x
    quat_real = ekf_data.pose.pose.orientation.w
    [ekf_yaw,ekf_pitch,ekf_roll] = common.quaternion_to_euler(quat_roll, quat_pitch, quat_yaw, quat_real)

    # Projecting point from the sonar onto a 2D-plane given range and angle of current ping
    bearing = (bearings[0]+bearings[1])/2.0
    scan_width = math.sin(bearing)*depth
    scan_height = math.sqrt(pow(depth,2) - pow(scan_width,2))                                         #math.sqrt(pow(range,2) - pow(scan_width,2))
    array_height = manta_x - (scan_height*scale)
    array_width = manta_y - (scan_width*scale)

    # Transforms ping from sonar_frame to auv_frame
    point=np.array([[int(array_height)], [int(array_width)]])
    center=np.array([[robot_x], [robot_y]])
    array_height,array_width = common.rotatePointAroundCenter(point,center,ekf_yaw)

    return (array_height,array_width)

# Extra functions for testing purposes
def checkForObject(image_data):
    # Transforms the image to HSV which is a color space easier to filter 
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(image_data, "bgr8")
    print(image.shape)
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
        pixel_width = bbox[0] + int((bbox[0]+bbox[2])/2)
        ''' 
        # Visualising bounding box for troubleshooting purposes
        cv2.rectangle(image,(bbox[0],bbox[1]),(bbox[0]+bbox[2],bbox[1]+bbox[3]),(0,255,0), 3)  
        cv2.imshow('Display',image)
        cv2.waitKey()
        '''
    if pixel_width != -1:
        return True, pixel_width
    return False, pixel_width

def find_sonar_depth(sonar_data,bearing):
    angle_increment = sonar_data.angle_increment
    min_angle = sonar_data.angle_min
    max_angle = sonar_data.angle_max
    scan_ranges = sonar_data.ranges
    b = scan_ranges[0]
    if bearing < min_angle:
        return scan_ranges[0]
    elif bearing > max_angle:
        return scan_ranges[-1]
    current_angle = sonar_data.angle_min
    depth = []
    for scan in scan_ranges:
        if ((current_angle < bearing) and (current_angle+angle_increment > bearing)) or ((current_angle > bearing) and (current_angle-angle_increment < bearing)):
            depth.append(scan)
        current_angle += angle_increment
    if len(depth) == 2:
        return (int(depth[0]+depth[1])/2.0)
    return False

def find_object_position(bearing,depth,ekf_data,width,height,scale,sonar_data): 
    # Getting mantas current position and heading
    robot_x = -int(ekf_data.pose.pose.position.x)*scale + int(height/2.0)
    robot_y = -int(ekf_data.pose.pose.position.y)*scale + int(width/2.0)
    quat_yaw = ekf_data.pose.pose.orientation.z
    quat_pitch = ekf_data.pose.pose.orientation.y
    quat_roll = ekf_data.pose.pose.orientation.x
    quat_real = ekf_data.pose.pose.orientation.w
    [ekf_yaw,ekf_pitch,ekf_roll] = misc.quaternion_to_euler(quat_roll, quat_pitch, quat_yaw, quat_real)

    # Projecting point from the sonar onto a 2D-plane given range and angle of current ping
    scan_width = math.sin(bearing)*depth
    scan_height = math.sqrt(pow(depth,2) - pow(scan_width,2))                                          #math.sqrt(pow(range,2) - pow(scan_width,2))
    array_height = int(scan_height*scale)
    array_width = int(scan_width*scale)

    # Transforms ping from sonar_frame to auv_frame
    point=np.array([[array_height], [array_width]])
    center=np.array([[robot_x], [robot_y]])
    array_height,array_width = misc.rotatePointAroundCenter(point,center,ekf_yaw)

    object_position_msg = ObjectPlacement()
    object_position_msg.object_x = array_height
    object_position_msg.object_y = array_width
    mapping_functions.imageCurrentScan(sonar_data,10,scale,(array_height,array_width))
    return object_position_msg