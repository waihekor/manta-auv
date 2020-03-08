import mapping_functions
import numpy as np
import math
from nav_msgs.msg import OccupancyGrid 
import rospy

def publishCurrentScan():
    pass

# Initiates a MapMsg with parameters from yaml file
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

# Fills in the data part of the msg and returns it 
def publishGlobalMap(sonar_data,ekf_data,map,WALL_WIDTH,SCALE,map_msg):
    map = mapping_functions.publishTheGlobalMap(sonar_data,ekf_data,map,WALL_WIDTH,SCALE)
    map_rotated = np.rot90(map)
    map_rotated = np.fliplr(map_rotated)
    map_grid = map_rotated.reshape(map.size,)*100
    map_msg.data = list(np.round(map_grid))        
    return map, map_msg