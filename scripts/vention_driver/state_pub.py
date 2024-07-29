#! /usr/bin/env python
##############################################################################
#      Title     : follow_joint_trajectory_action_server.py
#      Project   : vention_driver
#      Created   : 05/15/2024
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

#import os
#os.environ["ROS_NAMESPACE"] = "/dsr01dootion"

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

#TODO If namespace changes, must be reflected here. Or make it a variable pulled in somehow.
class FollowJointTrajectory():
    def __init__(self, argv):
        # Any vention related stuff goes here
        print("Vention noetic-side action server instantiated")

        # Initialize subscriber to action server+ Joint state publisher
        #! NOTE: THIS METHOD WILL ONLY WORK IN ROS 1. IF PORTING TO ROS 2, WITH THE CHANGES IN HOW ACTIONS WORK, A NEW METHOD WILL BE REQUIRED.
    
        # self.joint_state_pub = rospy.Publisher(
        #     "/dsr01dootion/joint_states", JointState, queue_size=10
        # )

        self.moveit_to_humble_pub = rospy.Publisher(
            '/dsr01dootion/dsr_joint_trajectory_controller/humble_command', JointTrajectory, queue_size=10
        )
        self.servo_to_arm_pub = rospy.Publisher(
            '/dsr01dootion/dsr_joint_trajectory_controller/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, queue_size=10
        )

        rospy.Subscriber("/dsr01dootion/dsr_joint_trajectory_controller/follow_joint_trajectory/goal", FollowJointTrajectoryActionGoal, self.moveit_callback)
        rospy.Subscriber("/joint_group_position_controller/command", JointTrajectory, self.servo_callback)
        rospy.Subscriber("/dsr01dootion/joint_states", JointState, self.updatestates_callback)
#        rospy.Subscriber("/surface_repair2/moveitPoseGoal", Pose, self.moveitToGoal)


        # Instantiate variables
        self.states = 0
        self.ventionstate = 0
        self.doosanstate = 0
        self.rate = rospy.Rate(2)

    #    moveit_commander.roscpp_initialize(sys.argv)
   #     self.robot = moveit_commander.RobotCommander()
  #      self.scene = moveit_commander.PlanningSceneInterface()
 #       group_name = "doosan_only"
#        self.move_group = moveit_commander.MoveGroupCommander(group_name)

        
    def publish_state(self):
        #TODO Is there any reason not to completely redo this, and instead have the vention wrapper intercept joint state messages and complete them?
        rospy.wait_for_service('jointservice')
        joint_state = JointState()
        while not rospy.is_shutdown():
            test = rospy.ServiceProxy('jointservice', Empty)
            #todo add error handling? note: Maybe not
            #todo need a way to not crash if the servo input is cancelled. need to track down where thats coming from
            #! This carries on to 
            test()
            #todo get joint states through vention driver
            # joint_state.header.stamp = rospy.get_rostime()
            # joint_state.name = ["tower_prismatic"]
            # joint_state.position = [0]
            # joint_state.velocity = [0]
            # joint_state.effort = [0]
            # self.joint_state_pub.publish(joint_state) 
            self.rate.sleep()

    def moveit_callback(self, data):
        self.moveit_to_humble_pub.publish(data.goal.trajectory)
        return
        

    # TODO for commands with vention only, populate message with current arm positions
    def servo_callback(self, data):
        message = FollowJointTrajectoryActionGoal()
        print(data.points[0].positions)
        message.goal.trajectory = data
        #todo states must be wrong...
        if (abs(sum(data.points[0].positions)-self.states) > 0.00001):
            print(abs(sum(data.points[0].positions)-self.states))
            self.servo_to_arm_pub.publish(message)
            print("Message published")
        return
    
    def updatestates_callback(self, data):
        #todo handle cases for arm only or vention only
        # if vention, update vention #, if doosan, update doosan #, then sum thte two at the end
        if len(data.name) == 1:
            self.ventionstate = sum(data.position)
        else:
            self.doosanstate = sum(data.position)
        self.states = self.ventionstate + self.doosanstate
        return
    

    #* A geometry_msgs/pose is receivied, published by ros2. That message should then be packaged up into moveit and 
#    def moveitToGoal(self, data):
        # * data = geometry_msgs/pose
        #todo should pose_goal just equal data?
       # pose_goal = geometry_msgs.msg.Pose()
       # pose_goal.orientation.w = data.orientation.w
       # pose_goal.position.x = data.position.x
      #  pose_goal.position.y = data.position.y
     #   pose_goal.position.z = data.position.z

    #    self.move_group.set_pose_target(pose_goal)
   #     success = self.move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
  #      self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
 #       self.move_group.clear_pose_targets()
 #       return
        


def main(argv):

    follow_joint_trajectory = FollowJointTrajectory(argv)
    joint_states_pub_thread = threading.Thread(
        target=follow_joint_trajectory.publish_state()
    )
    joint_states_pub_thread.start()


    while not rospy.is_shutdown():
        rospy.spin()

    joint_states_pub_thread.join()
    follow_joint_trajectory.disconnect()


if __name__ == "__main__":
    rospy.init_node("state_pub_node")
    
    reload(logging)
    argv = rospy.myargv(argv=sys.argv)
    if not main(argv[1:]):
        sys.exit(1)
