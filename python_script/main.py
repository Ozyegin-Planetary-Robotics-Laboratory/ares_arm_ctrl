#!/usr/bin/python3

import sys

import time
import rospy
import moveit_commander
import geometry_msgs.msg

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))

try:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("ares_arm_ctrl_node")
    robot = moveit_commander.RobotCommander()
    group_name = "manipulator"# robot.get_group_names()[0]
    print(group_name)
    move_group = moveit_commander.MoveGroupCommander(group_name)
    scene = moveit_commander.PlanningSceneInterface()
    move_group.set_goal_orientation_tolerance(1.0)
    move_group.set_goal_position_tolerance(0.1)
    move_group.set_goal_joint_tolerance(0.5)
    move_group.set_planning_time(35.0)
    # Add the ground plane as collision object
    plane_pose = geometry_msgs.msg.PoseStamped()
    plane_pose.header.frame_id = "world"
    plane_pose.pose.orientation.w = 1.0
    print(move_group.get_current_pose())
    #move_group.set_max_acceleration_scaling_factor(1)
    #move_group.set_max_velocity_scaling_factor(1)

    #joint_goal = move_group.get_current_joint_values()
    #joint_goal[0] = 0
    #joint_goal[1] = tau / 12
    #joint_goal[2] = tau / 12
    #joint_goal[3] = tau / 12

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    #move_group.go(joint_goal, wait=True)
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.3
    pose_goal.position.y = -0.04
    pose_goal.position.z = -0.7

    move_group.set_pose_target(pose_goal)

    # `go()` returns a boolean indicating whether the planning and execution was successful.
    success = move_group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets().
    move_group.clear_pose_targets()


except KeyboardInterrupt:
    sys.exit(0)
