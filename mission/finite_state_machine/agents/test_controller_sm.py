#!/usr/bin/env python

import rospy
import numpy as np

from smach import State, StateMachine, Sequence
from smach_ros import SimpleActionState, MonitorState, IntrospectionServer

# action message
import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from nav_msgs.srv import GetPlan, GetMap
from tf.transformations import quaternion_from_euler, euler_from_quaternion



class TaskManager():

    def __init__(self):




        rospy.init_node('test_contr_state_machine', anonymous=False)

        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        self.vehicle_odom = Odometry()
        self.sub_pose = rospy.Subscriber('/odometry/filtered', Odometry, self.positionCallback, queue_size=1)


        rospy.spin()

    def shutdown(self):
        rospy.loginfo("stopping the AUV...")
        rospy.sleep(10)

    
    def positionCallback(self, msg):
        self.vehicle_odom = msg
        self.time = msg.header.stamp.to_sec()
        global roll, pitch, yaw
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll,pitch,yaw) = euler_from_quaternion(orientation_list)

        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        

    def serviceSetup(self):

        print("waiting for service")
        rospy.wait_for_service('move_base_node/make_plan')
        print("finished waiting for service")
        get_plan = rospy.ServiceProxy('/move_base_node/make_plan', GetPlan)

        start = PoseStamped()
        goal = PoseStamped()
        start.header.frame_id = "manta/odom"
        start.pose.position.x = 0
        start.pose.position.y = 0
        start.pose.position.z = 0

        goal.header.frame_id = "manta/odom"
        goal.pose.position.x = -20
        goal.pose.position.y = 3
        goal.pose.position.z = 0

        tolerance = 0

        print("Start:")
        print(start)
        print("Goal: ")
        print(goal)


        plan_response = get_plan(start = start, goal = goal, tolerance = tolerance)
        print("Plan response type: ")
        print(type(plan_response))
    
        poses_arr = plan_response.plan.poses

        print("Lengde: array ", len(poses_arr))


if __name__ == '__main__':
	try:
		TaskManager()
	except rospy.ROSInterruptException:
		rospy.loginfo("Pathplanning state machine has been finished")