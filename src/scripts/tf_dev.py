#! /usr/bin/env python
# -*- coding: utf-8 -*-

# File:move_group_python.py
# Project: robotics_project
# Created Date: 04/06/2021
# Author: Nico Leuze, MAPR1
# -----
# Last Modified: XX.06.2021
# Modified By: Nico Leuze, MAPR1
# -----
# Munich University of Applied Science

import tf
import tf2_ros
import roslib
import rospy
import math
import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs
import numpy as np
from std_msgs.msg import String, Bool
from scipy.spatial.transform import Rotation as R
from moveit_commander.conversions import pose_to_list
import tf.transformations


class Listener():
    def __init__(self):

        self.pose_pub = rospy.Publisher("/pose_publisher/ur5_test", String, queue_size=1)
        self.trigger_sub = rospy.Subscriber("/trigger_transformation", Bool, self.trigger)

        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)

        # Get some random information from MoveIt Commander:
        self.group_names = self.robot.get_group_names()
        print "============ Available Planning Groups:", self.robot.get_group_names()
        print ""
        print "============ Robot State"
        print self.move_group.get_current_joint_values()
        print ""
        # print "============ Robot Pose"
        # print self.move_group.get_current_pose()
        # print ""

        # Misc variables
        # self.box_name = ''
        # self.robot = robot
        # self.scene = scene
        # self.move_group = move_group
        # self.display_trajectory_publisher = display_trajectory_publisher
        # self.group_names = group_names


    def trigger(self, data):
        flag = data.data
        # call transformation function
        if flag == True:
            self.transformation()
        else:
            print('XX')
        

    def transformation(self):

        print(self.group_names)
        print('Available Joint: ')
        print(self.move_group.get_joints())

        # tf_buffer = tf2_ros.Buffer()
        # listener = tf2_ros.TransformListener(tf_buffer)
        listener = tf.TransformListener()
        self.rate = rospy.Rate(100)
        iterations = 0

        # <<< tf2 implementation >>>

        # while not rospy.is_shutdown() and iterations < 5:
            # try:
                # transformation = tf_buffer.lookup_transform('base_link', 'wrist_3_link', rospy.Time())
                # print('1')
            # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                # self.rate.sleep()
                # continue
        
        # <<< tf implementation >>>

        listener.waitForTransform('/base_link', '/wrist_3_link', rospy.Time(), rospy.Duration(4.0))
        (trans, rot) = listener.lookupTransform('base_link', '/wrist_3_link', rospy.Time(0))
        for i in range(1):
            # print('transformation:')
            # print(transformation)

            # translation = transformation.transform.translation
            # rotation = transformation.transform.rotation
            print('Translation:')
            print(trans)
            print('Rotation:')
            print(rot, type(rot))
            # self.rot_eu = tf.transformations.euler_from_quaternion(self.rotation)

            rot = R.from_quat(rot)
            rot_1 = rot.as_rotvec()
            rot = rot_1.tolist()
            print('rot_euler: ')
            print(rot)

            transformation = list()
            transformation = trans + rot

            print('Transformation: ')
            print(transformation)
            print('get current rpy')
            print(self.move_group.get_current_rpy())
            print('joint_states')
            print(self.move_group.get_current_joint_values())
            print('joint_states_grad')
            joint_st = self.move_group.get_current_joint_values()
            # joint_st = math.radians(joint_st)
            # print(type(joint_st))
            joint_st = np.degrees(joint_st)
            print(joint_st)
            
            # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
            # <<< Notiz: Problem, dass Rx in URSim immer positiv dargestellt wird!! >>> 
            # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

            print('RX RY RZ: ')
            print(rot_1)
            # print('get current pose')
            # print(self.move_group.get_current_pose())
            # quaternion = (
            #     rotation.orientation.x,
            #     rotation.orientation.y,
            #     rotation.orientation.z,
            #     rotation.orientation.w
            # )

            # euler = tf.transformations.euler_from_quaternion(quaternion)

            # print('self.rot_eu: ')
            # print(euler[0])

            self.rate.sleep()
            iterations += 1


if __name__=='__main__':

    rospy.init_node('transformation_robotik', anonymous=True)
    Listener()
    try: 
        rospy.spin()
    except KeyboardInterrupt:
        print('Process aborted with Keyboard Interruption! Shutting down! ')


