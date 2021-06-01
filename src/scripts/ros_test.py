#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# File:ros_test.py
# Project: Robotik_Mapr1
# Created Date: 21/05/2021
# Author: Nico Leuze
# -----
# Last Modified: xx.xx.xxxx
# Modified By: Nico Leuze
# -----
# Copyright (c) 2021 Munich University of Applied Science

import roslib
import os
import sys
import rospy
import cv2
import time
import sys
import tf
import numpy
from std_msgs.msg import String, Bool, Int32, Int8
from sensor_msgs.msg import Image, JointState
from tf2_msgs.msg import TFMessage
from scipy.spatial.transform import Rotation as R


class GetStates():
    def __init__(self):
        rospy.Subscriber("/tf", TFMessage, self.tf_callback)
        rospy.Subscriber("/joint_states", JointState, self.jointstate_callback)
        rospy.Subscriber("/trigger_test", Bool, self.trigger)
        

    def trigger(self, data):
        print(data)
        trigger = data.data

        if trigger == True:
            self.flag = True
            print('\n JointStates: \n\n', js.position, type(js))
            print('\n TF \n\n', self.transformation, type(self.transformation))

            # <<< record path >>>
            time = list()
            Angle0 = list()
            Angle1 = list()
            Angle2 = list()
            Angle3 = list()
            Angle4 = list()
            Angle5 = list()

            t0 = 0.0
            time.append(t0)

            termination_condition = 0
            diff_time_condition = 0 
            iterations = 1
            listener = tf.TransformListener()

            while self.flag == True and termination_condition < 50:
                #print('Iteration: ', iterations)
                iterations += 1
                
                try:
                    (trans, rot) = listener.lookupTransform('/base', '/tool0_controller', rospy.Time(0))
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    #print('not working')
                    continue

                print('XXX')

                print('trans: ', trans)

                rot = R.from_quat(rot)
                rot = rot.as_rotvec()
                print('Rotation: ', rot)

    def tf_callback(self, data):

        self.transformation = data
        
        print('###### TRANSFORMS ######')
        print(data)
        rospy.sleep(3)

    def jointstate_callback(self, data):

        global js
        js = data

        print('\n\nJointstate Callback\n\n')
        print(data)
        rospy.sleep(3)







if __name__=="__main__":
    
    rospy.init_node('record_path_ros', anonymous=True)
    GetStates()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
