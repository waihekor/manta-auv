#!/usr/bin/env python
# Written by Kristoffer Rakstad Solberg, Student
# Copyright (c) 2020 Manta AUV, Vortex NTNU.
# All rights reserved.

import rospy
import numpy as np

from smach import State, StateMachine, Sequence
from smach_ros import SimpleActionState, MonitorState, IntrospectionServer

# action message
import actionlib
from vortex_msgs.msg import LosPathFollowingAction, LosPathFollowingGoal, LosPathFollowingResult, LosPathFollowingFeedback
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion

from tf.transformations import quaternion_from_euler

def make_los_goal(x_prev, y_prev, x_next, y_next, depth, speed=0.20, sphere_of_acceptance=0.5):
    
    los_goal = LosPathFollowingGoal()

    los_goal.prev_waypoint.x = x_prev
    los_goal.prev_waypoint.y = y_prev

    los_goal.next_waypoint.x = x_next
    los_goal.next_waypoint.y = y_next

    los_goal.desired_depth.z = depth

    los_goal.forward_speed.linear.x = speed
    los_goal.sphereOfAcceptance = sphere_of_acceptance

    return los_goal

def done_cb_state_machine(status, result):


    patrol = StateMachine([])

    with patrol:
        StateMachine.add(   'CP1',
                            SimpleActionState(  'los_path',
                                                LosPathFollowingAction,
                                                make_los_goal(2.0, 2.0, 4.0, 4.0, -0.5)),
                            transitions = { 'succeeded': 'CP2',
                                            'aborted': 'CP1',
                                            'preempted': 'CP2'})
        
        StateMachine.add(   'CP2',
                            SimpleActionState(  'los_path',
                                                LosPathFollowingAction,
                                                make_los_goal(4.0, 4.0, 2.0, 2.0, -0.5)),
                            transitions = { 'succeeded': 'CP1',
                                            'aborted': 'CP2',
                                            'preempted': 'CP1'})

    intro_server = IntrospectionServer(str(rospy.get_name()), patrol,'/SM_ROOT')
    intro_server.start()

    print("sdfsdfds")

    patrol.execute()

    intro_server.stop()

    rospy.loginfo('Finished done cb state machine')

    
        


def dummy_active_cb():
    rospy.loginfo("active cb")

def dummy_feedback_cb(feedback):
    rospy.loginfo(feedback)

if __name__ == '__main__':
    
    rospy.loginfo('before main')
    rospy.init_node('los_action_client')
    rospy.loginfo('starting main')
    """
    client = actionlib.SimpleActionClient('los_path',LosPathFollowingAction)
    rospy.loginfo("Waiting for LOS path action server...")
    wait = client.wait_for_server(rospy.Duration(15.0))
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    rospy.loginfo("Connected to LOS path server")
    rospy.loginfo("Starting goals achievements ...")
        
    _goal = LosPathFollowingGoal()

        # create line segment
    _goal.next_waypoint.x = 2.0 # 3.0
    _goal.next_waypoint.y = 2.0 # -2.0
    _goal.prev_waypoint.x = 0.0 # 0.0
    _goal.prev_waypoint.y = 0.0 # 0.0

        # set speed goal
    _goal.forward_speed.linear.x = 0.20

        # set desired depth
    _goal.desired_depth.z = -0.50

        # sphere of acceptance
    _goal.sphereOfAcceptance = 0.2

    rospy.loginfo("sending goal pose to Action Server")

    # Send goal
    client.send_goal(_goal, done_cb_state_machine, dummy_active_cb, dummy_feedback_cb)
    """
    done_cb_state_machine(1,1)


    
    
    rospy.spin()