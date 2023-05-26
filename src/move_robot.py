#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman
# Modified and adapted by: Jose Dos Santos (Spring 2023)

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Point32
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import time

#SET UP GPIO FOR ACTIVATING VACUUM GRIPPER
import RPi.GPIO as GPIO
# Pin Definitions
output_pin = 18  # BCM pin 18, BOARD pin 12
# Pin Setup:
GPIO.setmode(GPIO.BCM)  # BCM pin-numbering scheme from Raspberry Pi
# set pin as an output pin with optional initial state of LOW
GPIO.setup(output_pin, GPIO.OUT, initial=GPIO.LOW)

point = Point32()

# Main function, runs when a point is published to the '/point_in_robot_frame' topic
# Calls multiple custom MoveGroup methods to do planning and execution of motion to the coordinates determined in the transformation matrix calculation
def callback(data):
    
    global output_pin, point, move_robot, publisher
    print("ENTERED CALLBACK")
    point = data

    
    print("MOVING")
    move_robot.go_to_pose_goal()
    move_robot.go_down_to_pick()

    print("GO HOME")
    move_robot.go_to_home_pose()
    GPIO.output(output_pin, GPIO.LOW)

    print("SAFE TO MOVE AGAIN")
    safe_point = Point32()
    safe_point.x = 1
    publisher.publish(safe_point)


# Method to verify that goal and actual positions are within some tolerance
def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
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

# Define and initialize the MoveGroup class and its methods
class MoveGroupPythonInteface(object):
  """MoveGroupPythonInteface"""
  def __init__(self):
    super(MoveGroupPythonInteface, self).__init__()

    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_robot', anonymous=True)

    ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
    ## kinematic model and the robot's current joint states
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
    ## for getting, setting, and updating the robot's internal understanding of the surrounding world:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to a planning group (group of joints).  
    ## This interface can be used to plan and execute motions:     for the ABB IRB120 use "manipulator"
    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
    ## trajectories in Rviz:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    ## Create a ROS subscriber that looks for positions published to the /point_in_robot_frame topic
    rospy.Subscriber('/point_in_robot_frame', Point32, callback)

    global publisher
    publisher = rospy.Publisher('/safe_to_move', Point32, queue_size=0)

    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:
    planning_frame = move_group.get_planning_frame()
    print ("============ Planning frame: %s" % planning_frame)

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    print ("============ End effector link: %s" % eef_link)

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print ("============ Available Planning Groups:", robot.get_group_names())

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print ("============ Printing robot state")
    print (robot.get_current_state())
    print ("")
    ## END_SUB_TUTORIAL

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names
    

  def go_to_pose_goal(self):
    
    move_group = self.move_group
    print('CURRENT POSE')
    print(move_group.get_current_pose().pose)
    
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    pose_goal = geometry_msgs.msg.Pose()

    ## Define the position component of the pose goal using the desired object position determined previously
    pose_goal.position.x = point.x/1000       #this is in meters with respect to the robot frame
    pose_goal.position.y = point.y/1000
    pose_goal.position.z = (point.z)/1000

    ## Define the orientation quaternion of the pose goal. The following configuration results on a vertically oriented end-efector
    pose_goal.orientation.x = 0
    pose_goal.orientation.y = 1 
    pose_goal.orientation.z = 0
    pose_goal.orientation.w = 0 

    print(pose_goal)

    move_group.set_pose_target(pose_goal)
    print("Set pose target")
    ## Now, we call the planner to compute the plan and execute it.

    plan = move_group.plan(pose_goal)       #this plans ans displays the plan on rviz
    print("movegroup.plan")
    time.sleep(1)
    plan = move_group.go(wait=True)         #this does planning and execution
    print("Executed?")
    
    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    # Clear targets after planning with poses.
    move_group.clear_pose_targets()

    # Call "all_close" to verify the tolerance and success of the move
    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01) 
  
  def go_down_to_pick(self):

    global output_pin
    vertical_offset = 0.08
    move_group = self.move_group
    print('CURRENT POSE before going down')
    print(move_group.get_current_pose().pose)

    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    pose_goal = geometry_msgs.msg.Pose()

    #MOVE DOWN BY THE VERTICAL OFFSET TO PICK PACKAGE
    pose_goal = move_group.get_current_pose().pose

    pose_goal.position.z = pose_goal.position.z - vertical_offset

    print("Pose when going down")
    print(pose_goal)

    move_group.set_pose_target(pose_goal)
    print("Set pose target")
    ## Call the planner to compute the plan and execute it.

    plan = move_group.plan(pose_goal)       #this plans ans displays the plan on rviz
    print("movegroup.plan")
    time.sleep(1)
    plan = move_group.go(wait=True)         #this does planning and execution
    print("Executed?")
    
    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    # Clear targets after planning with poses.
    move_group.clear_pose_targets()

    #ACTUATE VACUUM GRIPPER
    GPIO.output(output_pin, GPIO.HIGH)

    #MOVE UP BY VERTICAL OFFSET FROM PICKING UP PACKAGE
    pose_goal = move_group.get_current_pose().pose

    pose_goal.position.z = pose_goal.position.z + vertical_offset

    move_group.set_pose_target(pose_goal)
    ## Call the planner to compute the plan and execute it.

    plan = move_group.plan(pose_goal)       #this plans ans displays the plan on rviz
    
    time.sleep(1)
    plan = move_group.go(wait=True)         #this does planning and execution
    
    
    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    # Clear targets after planning with poses.
    move_group.clear_pose_targets()

    # Call "all_close" to verify the tolerance and success of the move
    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01) 
  
  def go_to_home_pose(self):

    move_group = self.move_group
    print('CURRENT POSE')
    print(move_group.get_current_pose().pose)

    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    pose_goal = geometry_msgs.msg.Pose()

    # Define home (or drop) position
    pose_goal.position.x = -0.2       #this is in meters with respect to the robot frame
    pose_goal.position.y = -0.302
    pose_goal.position.z = 0.553

    pose_goal.orientation.x = 0
    pose_goal.orientation.y = 1 
    pose_goal.orientation.z = 0
    pose_goal.orientation.w = 0 

    print(pose_goal)

    move_group.set_pose_target(pose_goal)
    print("Set pose target")
    ## Call the planner to compute the plan and execute it.

    plan = move_group.plan(pose_goal)       #this plans ans displays the plan on rviz
    print("movegroup.plan")
    time.sleep(1)
    plan = move_group.go(wait=True)         #this does planning and execution
    print("Executed?")
    
    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    # Clear targets after planning with poses.
    move_group.clear_pose_targets()

    # Call "all_close" to verify the tolerance and success of the move
    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)

def main():
  try:
    print ("")
    print ("----------------------------------------------------------")
    print ("Welcome to the MoveIt MoveGroup Python Interface")
    print ("----------------------------------------------------------")
    print ("Press Ctrl-D to exit at any time")
    print ("")
    global move_robot
    move_robot = MoveGroupPythonInteface()
    print ("SPINNING")
    rospy.spin()

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

