import math
import misc
from vortex_msgs.msg import ObjectPlacement

def find_sonar_pos(pixel,a,b,c,d):
    if d != -100.0:
        return math.pow(float(pixel_width),3)*a + math.pow(float(pixel_width),2)*b + float(pixel_width)*c + d
    elif c != -100.0:
        return math.pow(float(pixel_width),2)*a + float(pixel_width)*b + c
    else:
        return float(pixel_width)*a + b

def find_sonar_depth(sonar_data,bearing)
    angle_increment = sonar_data.angle_increment
    current_angle = sonar_data.angle_min
    scan_ranges = sonar_data.ranges
    depth = []
    for scan in scan_ranges:
        if ((current_angle < bearing) and (current_angle+angle_increment > bearing)) or ((current_angle > bearing) and (current_angle-angle_increment < bearing)):
            depth.append(scan)
        current_angle += angle_increment
    if len(depth) == 2:
        return (int(depth[0]+depth[1])/2.0)
    return False

def find_object_position(bearing,depth,ekf_data,width,height,scale): 
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
    array_height = robot_x - int(scan_height*scale)
    array_width = robot_y - (scan_width*scale)

    # Transforms ping from sonar_frame to auv_frame
    point=np.array([[array_height], [array_width]])
    center=np.array([[robot_x], [robot_y]])
    array_height,array_width = misc.rotatePointAroundCenter(point,center,ekf_yaw)

    object_position_msg = ObjectPlacement()
    object_position_msg.object_x = array_height
    object_position_msg.object_y = array_width
    return object_position_msg




def checkForObject(image):
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