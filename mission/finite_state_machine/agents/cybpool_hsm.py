#!/usr/bin/env python
# Written by Kristoffer Rakstad Solberg, Student
# Copyright (c) 2020 Manta AUV, Vortex NTNU.
# All rights reserved.

import	rospy
import  math
import numpy as np
from    time import sleep
from	collections import OrderedDict
from	smach	import	State, StateMachine		
from    nav_msgs.msg import Odometry    
from	smach_ros	 import	SimpleActionState, IntrospectionServer	
from    move_base_msgs.msg  import  MoveBaseAction, MoveBaseGoal
from    vortex_msgs.msg import LosPathFollowingAction, LosPathFollowingGoal
from 	vortex_msgs.msg import PropulsionCommand
from 	geometry_msgs.msg import Wrench, Pose
from 	tf.transformations import euler_from_quaternion

# import mission plan
from finite_state_machine.mission_plan import *

# import object detection
from	vortex_msgs.msg import CameraObjectInfo

# camera centering controller
from autopilot.autopilot import CameraPID

# Imported help functions from src/finite_state_machine/
from    finite_state_machine import ControllerMode, WaypointClient, PathFollowingClient, SearchForTarget, TrackTarget

#ENUM
OPEN_LOOP           = 0
POSE_HOLD           = 1
HEADING_HOLD        = 2
DEPTH_HEADING_HOLD  = 3 
DEPTH_HOLD          = 4
POSE_HEADING_HOLD   = 5
CONTROL_MODE_END    = 6

class ControlMode(State):

    def __init__(self, mode):
        State.__init__(self, ['succeeded','aborted','preempted'])
        self.mode = mode
        self.control_mode = ControllerMode()

    def execute(self, userdata):

        # change control mode
        self.control_mode.change_control_mode_client(self.mode)
        rospy.loginfo('changed DP control mode to: ' + str(self.mode) + '!')
        return 'succeeded'

# A list of tasks to be done
task_list = {'docking':['transit'],
			 'gate':['searching','detect','camera_centering','path_planning','tracking', 'passed'],
			 'pole':['searching','detect','camera_centering','path_planning','tracking', 'passed']
			}

def update_task_list(target, task):
    task_list[target].remove(task)
    if len(task_list[target]) == 0:
        del task_list[target]



class TaskManager():

	def __init__(self):

		# init node
		rospy.init_node('pool_patrol', anonymous=False)

		# Set the shutdown fuction (stop the robot)
		rospy.on_shutdown(self.shutdown)

		# Initilalize the mission parameters and variables
		setup_task_environment(self)

		# Turn the target locations into SMACH MoveBase and LosPathFollowing action states
		nav_terminal_states = {}
		nav_transit_states = {}

		# DP controller
		for target in self.pool_locations.iterkeys():
			nav_goal = MoveBaseGoal() # Les mer her Kristian
			nav_goal.target_pose.header.frame_id = 'odom'
			nav_goal.target_pose.pose = self.pool_locations[target]
			move_base_state = SimpleActionState('move_base', MoveBaseAction,
												goal=nav_goal, 
												result_cb=self.nav_result_cb,
												exec_timeout=self.nav_timeout,
												server_wait_timeout=rospy.Duration(10.0))

			nav_terminal_states[target] = move_base_state

		# Path following
		for target in self.pool_locations.iterkeys():
			nav_goal = LosPathFollowingGoal()
			#nav_goal.prev_waypoint = navigation.vehicle_pose.position
			nav_goal.next_waypoint = self.pool_locations[target].position
			nav_goal.forward_speed.linear.x = self.transit_speed
			nav_goal.desired_depth.z = self.search_depth
			nav_goal.sphereOfAcceptance = self.search_area_size
			los_path_state = SimpleActionState('los_path', LosPathFollowingAction,
												goal=nav_goal, 
												result_cb=self.nav_result_cb,
												exec_timeout=self.nav_timeout,
												server_wait_timeout=rospy.Duration(10.0))

			nav_transit_states[target] = los_path_state

		""" Create individual state machines for assigning tasks to each target zone """

		# Create a state machine container for the orienting towards the gate subtask(s)
		sm_gate_tasks = StateMachine(outcomes=['found','unseen','missed','passed','aborted','preempted'])

		# Then add the subtask(s)
		with sm_gate_tasks:
			# if gate is found, pass pixel info onto TrackTarget. If gate is not found, look again
			StateMachine.add('SCANNING_OBJECTS', SearchForTarget('gate'), transitions={'found':'CAMERA_CENTERING','unseen':'BROADEN_SEARCH','passed':'','missed':''},
																	 remapping={'px_output':'object_px','fx_output':'object_fx','search_output':'object_search','search_confidence_output':'object_confidence'})

			StateMachine.add('CAMERA_CENTERING', TrackTarget('gate', self.pool_locations['gate'].position), transitions={'succeeded':'SCANNING_OBJECTS'},
														  remapping={'px_input':'object_px','fx_input':'object_fx','search_input':'object_search','search_confidence_input':'object_confidence'})

			StateMachine.add('BROADEN_SEARCH', TrackTarget('gate', self.pool_locations['gate'].position), transitions={'succeeded':'SCANNING_OBJECTS'},
														   remapping={'px_input':'object_px','fx_input':'object_fx','search_input':'object_search','search_confidence_input':'object_confidence'})


		# Create a state machine container for returning to dock
		sm_docking = StateMachine(outcomes=['succeeded','aborted','preempted'])

		# Add states to container
		


		with sm_docking:

			StateMachine.add('RETURN_TO_DOCK', nav_transit_states['docking'], transitions={'succeeded':'DOCKING_SECTOR','aborted':'','preempted':'RETURN_TO_DOCK'})
			StateMachine.add('DOCKING_SECTOR', ControlMode(POSE_HEADING_HOLD), transitions={'succeeded':'DOCKING_PROCEEDURE','aborted':'','preempted':''})
			StateMachine.add('DOCKING_PROCEEDURE', nav_terminal_states['docking'], transitions={'succeeded':'','aborted':'','preempted':''})

		""" Assemble a Hierarchical State Machine """

		# Initialize the HSM
		hsm_pool_patrol = StateMachine(outcomes=['succeeded','aborted','preempted','passed','missed','unseen','found'])

		# Build the HSM from nav states and target states

		with hsm_pool_patrol:


			""" Navigate to GATE in TERMINAL mode """
			StateMachine.add('TRANSIT_TO_GATE', nav_transit_states['gate'], transitions={'succeeded':'GATE_SEARCH','aborted':'DOCKING','preempted':'DOCKING'})

			""" When in GATE sector"""		
			StateMachine.add('GATE_SEARCH', sm_gate_tasks, transitions={'passed':'GATE_PASSED','missed':'DOCKING','aborted':'DOCKING'})		
			
			""" Transiting to gate """
			StateMachine.add('GATE_PASSED', ControlMode(OPEN_LOOP), transitions={'succeeded':'TRANSIT_TO_POLE','aborted':'DOCKING','preempted':'DOCKING'})
			StateMachine.add('TRANSIT_TO_POLE', nav_transit_states['pole'], transitions={'succeeded':'DOCKING','aborted':'DOCKING','preempted':'DOCKING'})

			""" When in POLE sector"""		
			#StateMachine.add('POLE_PASSING_TASK', sm_pole_tasks, transitions={'passed':'POLE_PASSING_TASK','missed':'RETURN_TO_DOCK','aborted':'RETURN_TO_DOCK'})		

			""" When aborted, return to docking """
			StateMachine.add('DOCKING', sm_docking, transitions={'succeeded':'','aborted':'','preempted':''})

		hsm_square_patrol = StateMachine(outcomes=['succeeded','aborted','preempted','passed','missed','unseen','found'])
		with hsm_square_patrol:
			"""Move to corner 1 """
			StateMachine.add('TRAVEL_P1',nav_transit_states['point1'],transitions={'succeeded':'DP_POS_P1','aborted':'TRAVEL_P1','preempted':'TRAVEL_P1'})
			StateMachine.add('DP_POS_P1',ControlMode(POSE_HOLD),transitions={'succeeded':'POS_P1','aborted':'TRAVEL_P1','preempted':'TRAVEL_P1'})
			StateMachine.add('POS_P1',nav_terminal_states['point1'],transitions={'succeeded':'TRANS_P2','aborted':'TRAVEL_P1','preempted':'TRAVEL_P1'})
			StateMachine.add('TRANS_P2',ControlMode(OPEN_LOOP),transitions={'succeeded':'TRAVEL_P2','aborted':'TRAVEL_P1','preempted':'TRAVEL_P1'})

			"""Move to corner 2 """
			StateMachine.add('TRAVEL_P2',nav_transit_states['point2'],transitions={'succeeded':'DP_POS_P2','aborted':'TRAVEL_P1','preempted':'TRAVEL_P1'})
			StateMachine.add('DP_POS_P2',ControlMode(POSE_HOLD),transitions={'succeeded':'POS_P2','aborted':'TRAVEL_P1','preempted':'TRAVEL_P1'})
			StateMachine.add('POS_P2',nav_terminal_states['point2'],transitions={'succeeded':'TRANS_P3','aborted':'TRAVEL_P1','preempted':'TRAVEL_P1'})
			StateMachine.add('TRANS_P3',ControlMode(OPEN_LOOP),transitions={'succeeded':'TRANSIT_TO_GATE','aborted':'TRAVEL_P1','preempted':'TRAVEL_P1'})
			
			""" Navigate to GATE in TERMINAL mode """
			StateMachine.add('TRANSIT_TO_GATE', nav_transit_states['gate'], transitions={'succeeded':'GATE_SEARCH','aborted':'TRAVEL_P1','preempted':'TRAVEL_P1'})
			StateMachine.add('GATE_SEARCH', sm_gate_tasks, transitions={'passed':'TRAVEL_P1','missed':'TRAVEL_P1','aborted':'TRAVEL_P1'})	

			"""Move to corner 3 """
			StateMachine.add('TRAVEL_P3',nav_transit_states['point3'],transitions={'succeeded':'DP_POS_P3','aborted':'TRAVEL_P1','preempted':'TRAVEL_P1'})
			StateMachine.add('DP_POS_P3',ControlMode(POSE_HOLD),transitions={'succeeded':'POS_P3','aborted':'TRAVEL_P1','preempted':'TRAVEL_P1'})
			StateMachine.add('POS_P3',nav_terminal_states['point3'],transitions={'succeeded':'TRANS_P1','aborted':'TRAVEL_P1','preempted':'TRAVEL_P1'})
			StateMachine.add('TRANS_P1',ControlMode(OPEN_LOOP),transitions={'succeeded':'TRAVEL_P1','aborted':'TRAVEL_P1','preempted':'TRAVEL_P1'})
			

		# Create and start the SMACH Introspection server

		intro_server = IntrospectionServer(str(rospy.get_name()),hsm_pool_patrol,'/SM_ROOT')
		intro_server.start()

		# Execute the state machine
		hsm_outcome = hsm_square_patrol.execute()
		intro_server.stop()

	def nav_result_cb(self, userdata, status, result):

		if status == GoalStatus.PREEMPTED:
			rospy.loginfo("Waypoint preempted")
		if status == GoalStatus.SUCCEEDED:
			rospy.loginfo("Waypoint succeeded")

	def shutdown(self):
		rospy.loginfo("stopping the AUV...")
		#sm_nav.request_preempt()
		rospy.sleep(10)


if __name__ == '__main__':

	try:
		TaskManager()
	except rospy.ROSInterruptException:
		rospy.loginfo("Mission pool patrol has been finished")