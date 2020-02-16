#!/usr/bin/python
import numpy as np
import cv2 as cv2
import rospy
from nav_msgs.msg import OccupancyGrid
from rospkg import RosPack



class MapPublisher:
    def __init__(self):

        ros_pack = RosPack()
        path_util = ros_pack.get_path('utilities')
        print(path_util)

        pic_name = "test.jpg"
        abs_path = path_util + "/src/" + pic_name
        print(abs_path)

        img = cv2.imread(abs_path)
        if img is None:
            print("Could not open file")

        print(type(img))
        print(img)
        

        self.map_msg = self.initiateMapMsg()


        for i in range(500*500):   
            self.map_msg.data[i] = 0
            if i%105 is 0:
                self.map_msg.data[i] = -1
            elif i%300 is 0:
                self.map_msg.data[i] = 100


        print(self.map_msg.data[0])


        self.map_pub_handle = rospy.Publisher('test/map', OccupancyGrid, queue_size=10)



        rospy.Timer(rospy.Duration(1.0/10.0),self.publishMapCallback)

        

    def initiateMapMsg(self):# written by Ambjorn Waldum
        map_msg = OccupancyGrid()
        map_msg.header.frame_id = 'manta/odom'
        resolution = 1
        width = 500
        height = 500
        map_msg.info.resolution = resolution
        map_msg.info.width = width
        map_msg.info.height = height
        map_msg.data = np.arange(width*height, dtype=np.int8)
        map_msg.info.origin.position.x = - width // 2 * resolution
        map_msg.info.origin.position.y = - height // 2 * resolution
        map_msg.header.stamp = rospy.Time.now()
        return map_msg

    def publishMapCallback(self, data):
        print("Publishing map")
        self.map_pub_handle.publish(self.map_msg)




#### INIT of node ##################################
if __name__ == '__main__':
    rospy.init_node('pre_map_publisher',anonymous=True)
    MapPublisher()
    rospy.spin()
###################################################
