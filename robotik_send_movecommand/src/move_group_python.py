#! /usr/bin/env python
# -*- coding: utf-8 -*-

# File:move_group_python.py
# Project: robotics_project
# Created Date: 29/05/2021
# Author: Nico Leuze, MAPR1
# -----
# Last Modified: 30.05.2021
# Modified By: Nico Leuze, MAPR1
# -----
# Munich University of Applied Science

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


def all_close(goal, actual, tolerance):

  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


class MoveGroupPython(object):

  def __init__(self):

    super(MoveGroupPython, self).__init__()

    ## Initialize `moveit_commander`, `RobotCommander` (kinematic model, joint states etc.),
    ## 'PlanningSceneInterface' (interaction with environment), 'MoveGroupCommander' (specifies
    ## the planning group - from where to where), 'Display Trajectory' (visualize trajectory in rviz)
    ## and a `rospy`_ node:

    moveit_commander.roscpp_initialize(sys.argv)

    rospy.init_node('move_group_python', anonymous=True)

    robot = moveit_commander.RobotCommander()

    scene = moveit_commander.PlanningSceneInterface()

    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    # Get some random information from MoveIt Commander:
    group_names = robot.get_group_names()
    print "============ Available Planning Groups:", robot.get_group_names()
    print ""
    print "============ Robot State"
    print move_group.get_current_joint_values()
    print ""
    print "============ Robot Pose"
    print move_group.get_current_pose()
    print ""
    print "============ Available Links: "
    print robot.get_link_names()
    print ""

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.group_names = group_names


  def go_to_joint_state(self):
    
    move_group = self.move_group

    ## Planning to a joint goal - first get actual state, then adjust them
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -pi/4
    joint_goal[2] = 0
    joint_goal[3] = -pi/2
    joint_goal[4] = 0
    joint_goal[5] = pi/3
    # <<< if 7 axes (endeffector) uncomment the line below >>>
    # joint_goal[6] = 0

    # execute the movement to the new goal
    move_group.go(joint_goal, wait=True)
    
    move_group.stop()

    # For testing:
    current_joints = move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


  def go_to_pose_goal(self):
    
    move_group = self.move_group

    ## Planning to a pose goal - first get actual pose, then adjust them
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.4
    pose_goal.position.y = 0.1
    pose_goal.position.z = 0.4

    move_group.set_pose_target(pose_goal)

    # execute the movement to the new pose
    plan = move_group.go(wait=True)
    
    move_group.stop()
    
    move_group.clear_pose_targets()

    # For testing:
    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)


  def plan_cartesian_path(self, scale=1):
    
    move_group = self.move_group
  
    ## Plan a Cartesian path directly by specifying a list of waypoints
    ## for the end-effector to go through. 
    waypoints = []

    wpose = move_group.get_current_pose().pose
    wpose.position.z -= scale * 0.1  # First move up (z)
    wpose.position.y += scale * 0.2  # and sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y -= scale * 0.1  # Third move sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    # Return the plan, execution see later
    return plan, fraction


  def execute_plan(self, plan):
    
    move_group = self.move_group

    ## Executing the plan from above
    
    move_group.execute(plan, wait=True)


  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    
    box_name = self.box_name
    scene = self.scene

    ## Ensuring Collision Updates Are Receieved
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0

      # Test if the box is in the scene.
      # Note that attaching the box will remove it from known_objects
      is_known = box_name in scene.get_known_object_names()

      # Test if we are in the expected state
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True
        
      rospy.sleep(0.1)
      seconds = rospy.get_time()

    return False


def main():
  try:
    print "============ Get some basic information about config"
    print ""
    tutorial = MoveGroupPython()

    print "============ Execute a movement using a joint state goal ..."
    raw_input()
    tutorial.go_to_joint_state()

    print "============ Execute a movement using a pose goal ..."
    raw_input()
    tutorial.go_to_pose_goal()

    print "============ Plan and display a Cartesian path ..."
    raw_input()
    cartesian_plan, fraction = tutorial.plan_cartesian_path()

    print "============ Execute a saved path ..."
    raw_input()
    tutorial.execute_plan(cartesian_plan)

    print "============ Finish!"

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
