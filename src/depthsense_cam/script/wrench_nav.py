#!/usr/bin/env python

"""
    grasp.py - Version 0.1 2015-11-05
    Use inverse kinemtatics to move the end effector to grab the wrench.
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    http://www.gnu.org/licenses/gpl.html
"""

import rospy
import sys
import moveit_commander
from geometry_msgs.msg import PoseArray, PoseStamped, PointStamped
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryActionResult
from actionlib_msgs.msg import *
import roslib
roslib.load_manifest("rosparam")
import rosparam

class wrench_nav():
    def __init__(self):
        # Name this node, it must be unique
	rospy.init_node('wrench_nav', anonymous=False)
        rospy.on_shutdown(self.cleanup) # Set rospy to execute a shutdown function when exiting
        self.flag = 0
        self.ct = 0
	self.loc_topic = rospy.get_param("~loc_topic", "/loc")
	self.error_topic = rospy.get_param("~error_topic", "/error")
	self.loc = None

        # Set up ROS subscriber callback routines
        rospy.Subscriber("/move_group/status", GoalStatusArray, self.callback, queue_size=1)
	rospy.Subscriber("/loc", PointStamped, queue_size=1)

        rospy.set_param('arm_prefix', 'ur5_arm_')
        rospy.set_param('reference_frame', '/base_link')
        rospy.loginfo("Moving arm to desired position")

	# Subscribe to the loc topic
        rospy.wait_for_message(self.loc_topic, PointStamped)
        
        # Use queue_size=1 so outdated poi and error messages are not piled up
        self.loc_subscriber = rospy.Subscriber(self.loc_topic, PointStamped, self.update_loc, queue_size=1)
        self.error_subscriber = rospy.Subscriber(self.error_topic, PointStamped, self.update_error, queue_size=1)

        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()

        # Initialize the move group for the ur5_arm
        self.arm = moveit_commander.MoveGroupCommander("ur5_arm")

        # Get the name of the end-effector link
        end_effector_link = self.arm.get_end_effector_link()
        rospy.loginfo(end_effector_link)

        # Initialize Necessary Variables
	self.reference_frame = rospy.get_param("~reference_frame", "/bsse_link")
        reference_frame = rospy.get_param("~reference_frame", "/base_link")
        #reference_frame = "ee_link"

        # Set the ur5_arm reference frame accordingly
        self.arm.set_pose_reference_frame(reference_frame)

        # Allow replanning to increase the odds of a solution
        self.arm.allow_replanning(True)

        # Allow some leeway in position (meters) and orientation (radians)
        self.arm.set_goal_position_tolerance(0.01)
        self.arm.set_goal_orientation_tolerance(0.01)

        # Set the target pose from the input
        self.target_pose = PoseStamped()
        self.target_pose.header.frame_id = reference_frame
        self.target_pose.header.stamp = rospy.Time.now()
        ee_pos = rospy.get_param('ee_position')
        self.target_pose.pose.position.x = ee_pos[0]
        self.target_pose.pose.position.y = ee_pos[1]
        self.target_pose.pose.position.z = ee_pos[2]

        # Get current joint position to use for planning
        jt = self.get_current_state()

        # Set the start state to the current state
        self.arm.set_start_state(jt)

        # Set the goal pose of the end effector to the stored pose
        self.arm.set_pose_target(self.target_pose, end_effector_link)

        # Plan the trajectory to the goal
        traj = self.arm.plan()
        traj_pts = len(traj.joint_trajectory.points)

        # Update current joint position for use in planning next time
        rospy.set_param('current_joint_state',traj.joint_trajectory.points[traj_pts-1].positions)

        if traj is not None:
            # Execute the planned trajectory
            self.arm.execute(traj)
	    
            # Pause for a second
            rospy.sleep(1.0)
            self.flag = 1
            #rospy.set_param('current_joint_state',jt)

	    # Correction step
	    jtc = self.get_current_state()
	    
	    # Set the start state to the current state
	    self.arm.set_start_state(jtc)
	    # Set the goal pose 
	    goal = self.get_goal(self.loc,self.target_pose)
	   
	    # Set the trajectory goal
	    self.arm.set_pose_target(goal, end_effector_link)
	    correct = self.arm.plan()
	    correct_pts = len(correct.joint_trajectory.points)

	    # Update current jont position for use in correction step
#	    rospy.set_param('current_joint_state', correct.joint_trajectory.points[correct_pts-1].position)
	
	    if correct is not None:
	   	# Execute the planned trajectory
		rospy.loginfo('Moving to wrench location')
		self.arm.execute(correct)
		# Pause for a second
		rospy.sleep(3.0)
		rospy.loginfo('Successfully moved')
		self.flag = 1
		
		
	    	# Approaching step
	    	jtp = self.get_current_state()
	    
	    	# Set the start state to the current state
	    	self.arm.set_start_state(jtp)
	    	# Set the goal pose 
		self.arm.shift_pose_target(0, 0.05, end_effector_link)
		rospy.loginfo('Approaching wrench')
		self.arm.go()
		rospy.sleep(3.0)
		rospy.loginfo('Successfully approached')
	   
	    else:
		rospy.loginfo('Unable to move to wrench location')

		

    # callback_feedback is used to store the feedback topic into the class to be
    # referenced by the other callback routines.

    def update_loc(self, loc):
        self.loc = loc
    
    def update_error(self, error):
        self.error = error

    def get_current_state(self):
	try:
	    cjs = rospy.get_param('current_joint_state')
	except:
	    cjs = [0,0,0,0,0,0]
	jts = RobotState()
	jts.joint_state.header.frame_id = '/base_link'
	jts.joint_state.name = ['front_left_wheel', 'front_right_wheel', 'rear_left_wheel', 'rear_right_wheel', 'ur5_arm_shoulder_pan_joint', 'ur5_arm_shoulder_lift_joint', 'ur5_arm_elbow_joint', 'ur5_arm_wrist_1_joint', 'ur5_arm_wrist_2_joint', 'ur5_arm_wrist_3_joint', 'left_tip_hinge', 'right_tip_hinge']
        jts.joint_state.position = [0,0,0,0,cjs[0],cjs[1],cjs[2],cjs[3],cjs[4],cjs[5],0,0]
	return	jts
    
    def get_goal(self, point, current_location):
        pose = PoseStamped()
        pose.header.frame_id = self.reference_frame
        pose.header.stamp = rospy.Time.now()     
        pose.pose.position.x = current_location.pose.position.x
        pose.pose.position.y = current_location.pose.position.y - point.point.y
        pose.pose.position.z = current_location.pose.position.z - point.point.z
        pose.pose.orientation = current_location.pose.orientation
        
        return pose

    def callback(self, data):
        if self.flag == 1:
            self.status = data.status_list[0].status
            
            if self.status == 3:
                rospy.sleep(5)
                rospy.set_param('move_arm_status','success')
                self.flag = 2
            else:
                if self.ct > 10:
                    rospy.set_param('move_arm_status','failure')
                    self.flag = 2
                else:
                    self.ct = self.ct + 1
                    rospy.sleep(0.5)
        if self.flag == 2:
            self.cleanup()
            rospy.signal_shutdown('Ending node.')

    def cleanup(self):
        rospy.loginfo("Stopping the robot")
    
        # Stop any current arm movement
        self.arm.stop()
        
        #Shut down MoveIt! cleanly
        rospy.loginfo("Shutting down Moveit!")
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == '__main__':
    try:
	wrench_nav()
    	rospy.spin()
    except KeyboardInterrupt:
	print "Shutting down the Wrench Navigation node."
	
