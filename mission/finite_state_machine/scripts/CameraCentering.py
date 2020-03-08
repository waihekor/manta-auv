#!/usr/bin/env python 
#Written by Kristoffer Rakstad Solberg, student
#Refined by Kristian Auestad, student
# Copyright (c) 2020 Manta AUV, Vortex NTNU.
# All rights reserved.


import rospy
import numpy as np
from time import sleep
from smach import State, StateMachine
from nav_msgs.msg import Odometry
from smach_ros import SimpleActionState,IntrospectionServer
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from vortex_msgs.msg import LosPathFollowingAction,LosPathFollowingGoal
from geometry_msgs.msg import Wrench,Pose
from tf.transformations import euler_from_quaternion

#Object detection message Tupe
from vortex_msgs.msg import CameraObjectInfo

#Controller
from autopilot.autopilot import CameraPID


class SearchForTarget(State):
  def __init__(self, target):
    #TODO: LEGG TIL OUTPUTKEYS
    State.__init__(self,outcomes=['found','unseen'],output_keys=[])

    self.target = target
    self.search_timeout = 30.0
    self.samlpling_time = 0.2
    self.timer = 0.0
    self.task_status = 'unseen'
    self.CameraPID = CameraPID()

    #Decide which object is relevant
    if target == 'gate':
      self.sub_object = rospy.Subscriber('/gate_midpoint',CameraObjectInfo,self.objectDetectionCallback,queue_size=1)
    elif target =='pole':
      self.sub_object = rospy.Subscriber('/pole_midpoint',CameraObjectInfo,self.objectDetectionCallback,queue_size=1)
    elif target =='bootlegger'
      self.sub_object = rospy.Subscriber('/bootlegger_midpoint',CameraObjectInfo,self.objectDetectionCallback,queue_size=1)
    elif target =='g-man'
      self.sub_object = rospy.Subscriber('/gman_midpoint',CameraObjectInfo,self.objectDetectionCallback,queue_size=1)
    elif target =='badge'
      self.sub_object = rospy.Subscriber('/badge_midpoint',CameraObjectInfo,self.objectDetectionCallback,queue_size=1)
    elif target =='tommy-gun'
      self.sub_object = rospy.Subscriber('/tommygun_midpoint',CameraObjectInfo,self.objectDetectionCallback,queue_size=1)

    #Get own pose
    #TODO:trengs en positionCallback funksjon?
    self.sub_pose = rospy.Subscriber('/odometry/filtered',Odometry,self.positionCallback,queue_size=1)
    #Publish thruster commands to manta
    self.pub_thust = rospy.Publisher('/manta/thruster_manager/input',Wrench,queue_size=1)
    self.thrust_msg = Wrench()

  def objectDetectionCallback(self,msg):
    self.object_pos_x = msg.pos_x
    self.object_pos_y = msg.pos_y
    self.object_frame_x = msg.frame_height #Usikker på denne
    #Ubrukt enda
    self.object_conf = msg.confidence
    #self.object_dist = msg.distance_to_pole
    pass

class TrackTarget(State):
  def __init__(self, search_target,search_area):
    #TODO:ADD INPUTKEYS
    State.__init__(self,outcomes=['succeded','aborted','preempted'],input_keys=['search_input'])

    #init controlelr
    self.CameraPID = CameraPID()

    self.pub_thust = rospy.Publisher('/manta/thruster_manager/input',Wrench,queue_size=1)
    self.thrust_msg = Wrench()

    self.sub_pose = rospy.Subscriber('/odometry/filtered',Odometry,self.positionCallback,queue_size=1)


  def positionCallback(self,msg):
    self.manta_odom = msg
    self.time = msg.header.stamp.to_sec() # Is this used?
    

    #import roll pitch yaw and update them
    global roll, pitch, yaw
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x,orientation_q.y,orientation_q.z,orientation_q.w]
    (roll,pitch,yaw) = euler_from_quaternion(orientation_list)

    self.psi = yaw    
    
    
    pass
    

  def alignWithTarget(self,object_frame_x,object_pos_x,search_input,object_conf):
    #Separate Controller keeping manta at 0.5 meter depth
    tau_heave = self.CameraPID.depthController(-0.5,self.manta_odom.pose.pose.position.z,self.time)
    self.thrust_msg.force.z = tau_heave

    #Set screen center as target
    target_center_screen = object_frame_x * 0.5 #adjust this to align of center

    if search_input == 'found' and object_conf >=0.9: #Think object_conf is conf from object detection. 
      tau_sway = self.CameraPID.swayController(target_center_screen,object_pos_x,self.time)#sway towards center of screen
      tau_speed = selc.CameraPID.speedController(0.1,self.manta_odom.twist.twist.linear.x,self.time) #move slowly forward
      
      tau_heading = self.CameraPID.headingController(self.psi,self.psi,self.time)

      #publish new values
      self.thrust_msg.torque.z = tau_heading
      self.thrust_msg.force.x = tau_speed
      self.thrust_msg.force.y = tau_sway
    
    else:

      tau_speed = self.CameraPID.speedController(0.3,self.manta_odom.twist.twist.linear.x, self.time)
      tau_heading = self.headingController(0.1,self.psi,self.time) #0.1 is rad searching dir ? 
      tau_sway = 0.0

      #publish new values
      self.thrust_msg.torque.z = tau_heading
      self.thrust_msg.force.x = tau_speed
      self.thrust_msg.force.y = tau_sway
