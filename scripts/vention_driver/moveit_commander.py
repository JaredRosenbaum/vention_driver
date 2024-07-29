#! /usr/bin/env python
##############################################################################
#      Title     : moveit_commander.py
#      Project   : vention_driver
#      Created   : 07/29/2024
#      Author    : Jared Rosenbaum
#      Copyright : Copyright© The University of Texas at Austin, 2024-2030. All
#      rights reserved.
#
#          All files within this directory are subject to the following, unless
#          an alternative license is explicitly included within the text of
#          each file.
#
#          This software and documentation constitute an unpublished work
#          and contain valuable trade secrets and proprietary information
#          belonging to the University. None of the foregoing material may be
#          copied or duplicated or disclosed without the express, written
#          permission of the University. THE UNIVERSITY EXPRESSLY DISCLAIMS ANY
#          AND ALL WARRANTIES CONCERNING THIS SOFTWARE AND DOCUMENTATION,
#          INCLUDING ANY WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
#          PARTICULAR PURPOSE, AND WARRANTIES OF PERFORMANCE, AND ANY WARRANTY
#          THAT MIGHT OTHERWISE ARISE FROM COURSE OF DEALING OR USAGE OF TRADE.
#          NO WARRANTY IS EITHER EXPRESS OR IMPLIED WITH RESPECT TO THE USE OF
#          THE SOFTWARE OR DOCUMENTATION. Under no circumstances shall the
#          University be liable for incidental, special, indirect, direct or
#          consequential damages or loss of profits, interruption of business,
#          or related expenses which may arise from use of software or
#          documentation, including but not limited to those resulting from
#          defects in software and/or documentation, or loss or inaccuracy of
#          data of any kind.
##############################################################################
import logging
import sys
import threading
from importlib import reload

import os
os.environ["ROS_NAMESPACE"] = "/dsr01dootion"

import actionlib
import rospy
from control_msgs.msg import (FollowJointTrajectoryAction,
                              FollowJointTrajectoryFeedback,
                              FollowJointTrajectoryResult,
                              FollowJointTrajectoryActionGoal)
# from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty
from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import Pose

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
# from moveit_commander.conversions import pose_to_list



#TODO If namespace changes, must be reflected here. Or make it a variable pulled in somehow.
class MoveitThread():
    def __init__(self, argv):
        # Any vention related stuff goes here
        print("Moveit commander active")

        rospy.Subscriber("/surface_repair2/moveitPoseGoal", Pose, self.moveitToGoal)


        # Instantiate variables
        self.rate = rospy.Rate(2)

        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        group_name = "doosan_only"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)

    
    

    #* A geometry_msgs/pose is receivied, published by ros2. That message should then be packaged up into moveit and 
    def moveitToGoal(self, data):
        # * data = geometry_msgs/pose
        #todo should pose_goal just equal data?
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = data.orientation.w
        pose_goal.position.x = data.position.x
        pose_goal.position.y = data.position.y
        pose_goal.position.z = data.position.z

        self.move_group.set_pose_target(pose_goal)
        success = self.move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        self.move_group.clear_pose_targets()
        return
        


def main():
    # Initialize the ROS node
    rospy.init_node('moveit_node', anonymous=True)

    # Create an instance of your MoveitThread class
    moveit_thread = MoveitThread(sys.argv)

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    main()