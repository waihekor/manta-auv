#!/usr/bin/env python

import rospy
import numpy as np

from smach import State, StateMachine, Sequence
from smach_ros import SimpleActionState, MonitorState, IntrospectionServer

# action message
import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped, Wrench
from nav_msgs.msg import OccupancyGrid, Odometry
from nav_msgs.srv import GetPlan, GetMap
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Twist


class TaskManager():

    def __init__(self):


        rospy.init_node('test_contr_state_machine', anonymous=False)

        self.x = 0
        self.y = 0
        self.z = 0

        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        self.path = []

        self.vehicle_odom = Odometry()
        self.sub_pose = rospy.Subscriber('/odometry/filtered', Odometry, self.positionCallback, queue_size=1)
        self.sub_cmd_vel = rospy.Subscriber('/cmd_vel', Twist, self.cmdvelCallback, queue_size=1)
        self.pub_thrust = rospy.Publisher('/manta/thruster_manager/input', Wrench, queue_size=1)



        print("Rospy spin")
        rospy.spin()
        print("Finished TaskManager")

    def shutdown(self):
        rospy.loginfo("stopping the AUV...")
        rospy.sleep(10)

    def cmdvelCallback(self, msg):
        print(msg)
        x_vel = msg.linear.x
        y_vel = msg.linear.y
        
        z_twist = msg.angular.z

        cmdWrench = Wrench()
        cmdWrench.force.x = x_vel*20
        cmdWrench.force.y = y_vel*20
        cmdWrench.torque.z = z_twist*15

        print("Forcex: ", cmdWrench.force.x, " Forcey: ", cmdWrench.force.y, " Torquez: ", cmdWrench.torque.z)

        self.pub_thrust.publish(cmdWrench)

    
    def positionCallback(self, msg):
        self.vehicle_odom = msg
        self.time = msg.header.stamp.to_sec()
        global roll, pitch, yaw
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll,pitch,yaw) = euler_from_quaternion(orientation_list)

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z

        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw


    def goForward(self):
        testWrench = Wrench()
        #testWrench.force.x = 30
        #testWrench.torque.z = 100
        self.pub_thrust.publish(testWrench)

   

if __name__ == '__main__':
	try:
		TaskManager()
	except rospy.ROSInterruptException:
		rospy.loginfo("Pathplanning state machine has been finished")