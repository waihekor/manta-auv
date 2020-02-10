#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid

if __name__ == '__main__':
    rospy.init_node("get_map_node")
    print("Started main")


    """
    NodeHandle n
    ServiceClient mGetMapClient = n.serviceClient<nav_msgs::GetMap>(std::string("dynamic_map"))
    nav_msgs::GetMap srv;
    if(!mGetMapClient.call(srv))
    {
        ROS_INFO("Could not get a map.");
        return false;
    }
    """


    rospy.spin()
	