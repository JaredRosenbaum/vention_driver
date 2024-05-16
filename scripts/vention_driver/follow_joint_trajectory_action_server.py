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

import actionlib
import rospy
from control_msgs.msg import (FollowJointTrajectoryAction,
                              FollowJointTrajectoryFeedback,
                              FollowJointTrajectoryResult,
                              FollowJointTrajectoryActionGoal)
# from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty


#TODO If namespace changes, must be reflected here. Or make it a variable pulled in somehow.
class FollowJointTrajectory():
    def __init__(self, argv):
        # Any vention related stuff goes here
        print("init")

        # Initialize subscriber to action server+ Joint state publisher
        #! NOTE: THIS METHOD WILL ONLY WORK IN ROS 1. IF PORTING TO ROS 2, WITH THE CHANGES IN HOW ACTIONS WORK, A NEW METHOD WILL BE REQUIRED.
    
        self.joint_state_pub = rospy.Publisher(
            "/dsr01dootion/joint_states", JointState, queue_size=10
        )

        rospy.Subscriber("/dsr01dootion/dsr_joint_trajectory_controller/follow_joint_trajectory/goal", FollowJointTrajectoryActionGoal, self.callback)

        

        # Start action server

        # Initialize any variables


        self.rate = rospy.Rate(10)
    
        
    def follow_joint_trajectory_feedback(self):
        while self.feedback_publish_flag:
            # get joint states
            #TODO rest
            self.feedback.header.stamp = rospy.Time()

            # self.prismatic_action_server.publish_feedback(self.feedback) #todo uncomment once populated properly
            self.rate.sleep()

    def publish_state(self):
        print("waiting for service")
        rospy.wait_for_service('jointservice')
        print("found service")
        test = rospy.ServiceProxy('jointservice', Empty)
        print("Proxy instantiated")
        test()
        print("Past call")
        joint_state = JointState()
        while not rospy.is_shutdown():
            #todo get joint states through vention driver
            joint_state.header.stamp = rospy.get_rostime()
            joint_state.name = ["tower_prismatic"]
            joint_state.position = [0]
            joint_state.velocity = [0]
            joint_state.effort = [0]


            self.joint_state_pub.publish(joint_state)
            self.rate.sleep()

    def callback(self, data):
        traj_point_positions = []
        traj_point_velocities = []
        time_since_ref = []

        for i in range(0, len(data.goal.trajectory.points)):
            for j in range(0, len(data.goal.trajectory.joint_names)):
                if data.goal.trajectory.joint_names[j] == "tower_prismatic":
                    traj_point_positions.append(data.goal.trajectory.points[i].positions[j])
                    traj_point_velocities.append(data.goal.trajectory.points[i].velocities[j])
                    time_since_ref.append(data.goal.trajectory.points[i].time_from_start.to_sec())

        for i in range(0,len(traj_point_positions)):
            print(traj_point_positions[i])
            print(traj_point_velocities[i])
            print(time_since_ref[i])
            print('----')

        goal = traj_point_positions[-1]
        target_vel = traj_point_velocities[-1]
        target_time = time_since_ref[-1]


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
    rospy.init_node("follow_joint_trajectory_node")
    reload(logging)
    argv = rospy.myargv(argv=sys.argv)
    if not main(argv[1:]):
        sys.exit(1)
