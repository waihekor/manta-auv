import mapping_functions
import misc
import numpy as np
import math
from nav_msgs.msg import OccupancyGrid 
import rospy

def publishCurrentScan():
    pass

def initiateMapMsg():
     # Initiating map_msg, resolution, width and height should be feed in through a YAML file
    map_msg = OccupancyGrid()
    map_msg.header.frame_id = 'map'
    resolution = 0.2
    width = 500
    height = 500
    map_msg.info.resolution = resolution
    map_msg.info.width = width
    map_msg.info.height = height
    map_msg.data = range(width*height)
    map_msg.info.origin.position.x = - width // 2 * resolution
    map_msg.info.origin.position.y = - height // 2 * resolution
    map_msg.header.stamp = rospy.Time.now()
    map_msg.header.frame_id = "manta/odom"
    return map_msg

def publishGlobalMap(sonar_data,ekf_data,map,WALL_WIDTH,SCALE,map_msg):
    map = mapping_functions.imageGlobalMap(sonar_data,ekf_data,map,WALL_WIDTH,SCALE)
    map_rotated = np.rot90(map)
    map_rotated = np.fliplr(map_rotated)
    
    map_grid = map_rotated.reshape(map.size,)*100
    map_msg.data = list(np.round(map_grid))


    '''
    for i in range(250000):
        if map.flat[i] == 0:
            map_msg.data[i] = 100
        elif map.flat[i] == 1:
            map_msg.data[i] = -1
        else:
            map_msg.data[i] = 0
    '''
        
    return map, map_msg