#!/usr/bin/env python

import rospy
import numpy as np

from smach import State, StateMachine, Sequence
from smach_ros import SimpleActionState, MonitorState, IntrospectionServer

# action message
import actionlib
from vortex_msgs.msg import LosPathFollowingAction, LosPathFollowingGoal, LosPathFollowingResult, LosPathFollowingFeedback
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from nav_msgs.srv import GetPlan, GetMap
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from visualization_msgs.msg import Marker, MarkerArray


class Init_state(State):
    def __init__(self):
        State.__init__(self, outcomes = ["first_state"])
    
    def execute(self, userdata):
        print("Starting state Init_state")

        return "first_state"

class Init_point_1(State):
    
    def __init__(self):
        State.__init__(self, outcomes = ["succeded"])
    
    def execute(self, userdata):
        print("Starting state init_point_1")
        return "succeded"

class Init_point_0(State):
    
    def __init__(self, service_func):
        State.__init__(self, outcomes = ["succeded"])

        self.service_func = service_func
    
    def execute(self, userdata):
        self.service_func()
        print("Starting state init_point_0")
        return "succeded"


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

        self.psi = 0


        rospy.init_node('pathplanning_sm', anonymous=False)

        self.marker_arr = MarkerArray()

        self.vehicle_odom = Odometry()
        self.sub_pose = rospy.Subscriber('/odometry/filtered', Odometry, self.positionCallback, queue_size=1)
        subscrib_map = rospy.Subscriber('/map', OccupancyGrid, self.mapCB, queue_size=1)

        #self.marker_pub = rospy.Publisher('/testing/markerArray', MarkerArray, queue_size=50)


        hsm = StateMachine(outcomes=['finished statemachine'])
        
        with hsm:
            StateMachine.add('INIT', Init_state(), transitions={"first_state": 'INIT_GO_TO_POINT_1'})
            StateMachine.add('INIT_GO_TO_POINT_1', Init_point_1(), transitions={ "succeded": 'GO_TO_POINT_1' })
            StateMachine.add(   'GO_TO_POINT_1',
                        SimpleActionState(  'los_path',
                                            LosPathFollowingAction,
                                            make_los_goal(0.0, 0.0, 2, 0.0, -0.5)),
                                            transitions = { "succeeded": 'INIT_GO_TO_POINT_0',
                                                            "preempted": 'INIT_GO_TO_POINT_0', 
                                                            "aborted": 'INIT_GO_TO_POINT_0' })   
            StateMachine.add('INIT_GO_TO_POINT_0', Init_point_0(self.serviceSetup), transitions={ "succeded": 'GO_TO_POINT_0'})      
            StateMachine.add(   'GO_TO_POINT_0',
                         SimpleActionState( 'los_path',
                                            LosPathFollowingAction,
                                            make_los_goal(2, 0.0, 0.0, 0.0, -0.5)),
                                            transitions = { 'succeeded': 'INIT_GO_TO_POINT_1',
                                                            "preempted": 'INIT_GO_TO_POINT_1', 
                                                            "aborted": 'INIT_GO_TO_POINT_1' })  
            
        hsm.execute()

        


        intro_server = IntrospectionServer(str(rospy.get_name()), hsm,'/SM_ROOT')
        intro_server.start()
        #patrol.execute()
        print("State machine execute finished")
        intro_server.stop()
        rospy.spin()

    def shutdown(self):
        rospy.loginfo("stopping the AUV...")
        rospy.sleep(10)

    def mapCB(self, msg):
        self.occ_grid = msg
        self.get_grid_index(self.occ_grid, self.vehicle_odom)

    
    def positionCallback(self, msg):
        self.vehicle_odom = msg
        self.time = msg.header.stamp.to_sec()
        global roll, pitch, yaw
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll,pitch,yaw) = euler_from_quaternion(orientation_list)
        self.psi = yaw
        



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

        """
        self.marker_arr = MarkerArray()
        
        id = 0

        for poses in poses_arr:
            id = id + 1
            marker = Marker()
            marker.id = id
            marker.header.frame_id = "manta/odom"
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.pose.orientation.w = 1.0

            print(poses.pose.position)

            marker.pose.position.x = poses.pose.position.x
            marker.pose.position.y = poses.pose.position.y
            marker.pose.position.z = poses.pose.position.z
            self.marker_arr.markers.append(marker)
        """
        
            
            

"""
    def drawMarkersTest(self):

        print("length of marker array: ", len(self.marker_arr.markers))
        self.marker_pub.publish(self.marker_arr)
"""
    



    
    


if __name__ == '__main__':
	try:
		TaskManager()
	except rospy.ROSInterruptException:
		rospy.loginfo("Pathplanning state machine has been finished")