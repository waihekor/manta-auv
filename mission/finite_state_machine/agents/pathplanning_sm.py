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


class Foo(State):
    def __init__(self):
        State.__init__(self, outcomes = ["outcome1", "outcome3"])
    
    def execute(self, userdata):
        print("Starting state Foo")
        rospy.sleep(3)
        print("Waited 3 seconds")
        return "outcome3"

class Bar(State):
    
    def __init__(self):
        State.__init__(self, outcomes = ["outcome2"])
    
    def execute(self, userdata):
        print("Starting state Bar")
        rospy.sleep(2)
        print("Waited 2 seconds")
        return "outcome2"

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

        

        self.vehicle_odom = Odometry()
        #self.sub_pose = rospy.Subscriber('/odometry/filtered', Odometry, self.positionCallback, queue_size=1)
        subscrib_map = rospy.Subscriber('/map', OccupancyGrid, self.mapCB, queue_size=1)


        patrol = StateMachine(outcomes = ["outcome_patrol"])

        print("sm init")


        with patrol:
            StateMachine.add(   'CP1',
                                SimpleActionState(  'los_path',
                                                    LosPathFollowingAction,
                                                    make_los_goal(0.0, 0.0, -9.0, 0.0, -0.5)),
                                transitions = { 'succeeded': 'CP2',
                                                'aborted': 'CP1',
                                                'preempted': 'CP2'})
            
            StateMachine.add(   'CP2',
                                SimpleActionState(  'los_path',
                                                    LosPathFollowingAction,
                                                    make_los_goal(-9.0, 0.0, 0.0, 0.0, -0.5)),
                                transitions = { 'succeeded': 'CP1',
                                                'aborted': 'CP2',
                                                'preempted': 'CP1'})

        #rospy.on_shutdown(self.shutdown)

        hsm = StateMachine([])
        with hsm:
            StateMachine.add('FOO', Foo(), transitions={"outcome1": 'BAR', "outcome3": 'BAS'})
            StateMachine.add('BAR', Bar(), transitions={"outcome2": 'FOO'})
            StateMachine.add('BAS', patrol, transitions={"outcome_patrol": 'BAR'})
            
        hsm.execute()

        


        intro_server = IntrospectionServer(str(rospy.get_name()), patrol,'/SM_ROOT')
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
        

    def get_grid_index(self, occ_map, manta_pos):
        width = occ_map.info.width
        height = occ_map.info.height
        resolution = occ_map.info.resolution
        origin = occ_map.info.origin.position

        
        
        manta_grid_pos_x = manta_pos.pose.pose.position.x / resolution
        manta_grid_pos_y = manta_pos.pose.pose.position.y / resolution

    def serviceSetup(self):

        print("waiting for service")
        rospy.wait_for_service('move_base_node/make_plan')
        print("finished waiting for service")
        get_plan = rospy.ServiceProxy('/move_base_node/make_plan', GetPlan)

        start = PoseStamped()
        goal = PoseStamped()
        start.header.frame_id = "world"
        start.pose.position.x = 0
        start.pose.position.y = 0
        start.pose.position.z = -0.5

        goal.pose.position.x = 3
        goal.pose.position.y = 0
        goal.pose.position.z = -0.5

        tolerance = 0
        plan_response = get_plan(start = start, goal = goal, tolerance = tolerance)
        print("Plan response type: ")
        print(type(plan_response))
    
        poses_arr = plan_response.plan.poses

        print("Lengde: array ", len(poses_arr))

        for poses in poses_arr:
            print("hei")
            print(poses)

        





    

    
        


if __name__ == '__main__':
    
	try:
		TaskManager()
	except rospy.ROSInterruptException:
		rospy.loginfo("Pathplanning state machine has been finished")