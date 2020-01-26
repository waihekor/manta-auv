#!/usr/bin/env python

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


class TaskManager():

    def __init__(self):


        rospy.init_node('pathplanning_sm', anonymous=False)

        #rospy.on_shutdown(self.shutdown)

        patrol = StateMachine([])

        print("sm init")


        with patrol:
            StateMachine.add(   'CP1',
                                SimpleActionState(  'los_path',
                                                    LosPathFollowingAction,
                                                    make_los_goal(0.0, 0.0, 4.0, 4.0, -0.5)),
                                transitions = { 'succeeded': 'CP2',
                                                'aborted': 'CP1',
                                                'preempted': 'CP2'})
            
            StateMachine.add(   'CP2',
                                SimpleActionState(  'los_path',
                                                    LosPathFollowingAction,
                                                    make_los_goal(4.0, 4.0, 0.0, 0.0, -0.5)),
                                transitions = { 'succeeded': 'CP1',
                                                'aborted': 'CP2',
                                                'preempted': 'CP1'})


        intro_server = IntrospectionServer(str(rospy.get_name()), patrol,'/SM_ROOT')
        intro_server.start()
        patrol.execute()
        print("State machine execute finished")
        intro_server.stop()
        rospy.spin()

    def shutdown(self):
        rospy.loginfo("stopping the AUV...")
        rospy.sleep(10)


if __name__ == '__main__':
    
	try:
		TaskManager()
	except rospy.ROSInterruptException:
		rospy.loginfo("Pathplanning state machine has been finished")