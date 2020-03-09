#!/usr/bin/env python
# coding: UTF-8

import rospy
from smach import StateMachine, Sequence
from std_msgs.msg import String
from math import pi
from helper import dp_move, los_move, patrol_sequence


rospy.init_node('simtest_fsm')

patrol_sm = patrol_sequence([
    dp_move(0, 0),
    los_move(8, 2),
    dp_move(8,2)
])

try:
    patrol_sm.execute()

except:
    rospy.loginfo("Pooltest is stoppted")   
