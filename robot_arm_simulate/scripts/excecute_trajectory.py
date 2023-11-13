#! /usr/bin/env python
import sys
import numpy as np
import rospy
from geometry_msgs.msg import PointStamped
import moveit_commander
import moveit_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('trajectory_control', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("robot_arm")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
mesh_nodes_publisher = rospy.Publisher('/mesh_nodes', PointStamped, queue_size=5)

goals = []
goal_node = PointStamped()
sequence = 1

with open('/home/user/robot_ws/src/robot_arm_simulate/src/joint_values.txt', 'r') as filehandle:

    filecontents = filehandle.readlines()
    for line in filecontents:
        q1, q2, q3 = line[:-1].split("\t")
        q1 = float(q1)
        q2 = float(q2)
        q3 = float(q3)
        goals.append([q1, q2, q3])

### HOME ###
joint_goal = group.get_current_joint_values()
joint_goal[0] = 0.0
joint_goal[1] = 0.0
joint_goal[2] = 0.0
group.go(joint_goal, wait=True)
group.stop()
print("home_pose", joint_goal)

actual_pose = group.get_current_pose().pose
goal_node.header.seq = sequence
goal_node.header.stamp = rospy.Time.now()
goal_node.header.frame_id = "world"
goal_node.point = actual_pose.position
mesh_nodes_publisher.publish(goal_node)
rospy.loginfo(goal_node)

### MESH ###
iteration = 0
for goal in goals:
    joint_goal = goal
    print(joint_goal)
    group.go(joint_goal, wait=True)
    group.stop()
    
    sequence = sequence + 1
    actual_pose = group.get_current_pose().pose
    goal_node.header.seq = sequence
    goal_node.header.stamp = rospy.Time.now()
    goal_node.header.frame_id = "world"
    goal_node.point = actual_pose.position
    mesh_nodes_publisher.publish(goal_node)
    rospy.loginfo(goal_node)

    if iteration == 0: real_position = np.array([[actual_pose.position.y, actual_pose.position.z]])
    else : real_position = np.append(real_position, [[actual_pose.position.y, actual_pose.position.z]], axis=0)
    iteration = iteration + 1

### HOME ###
joint_goal[0] = 0.0
joint_goal[1] = 0.0
joint_goal[2] = 0.0
group.go(joint_goal, wait=True)
group.stop()

actual_pose = group.get_current_pose().pose
goal_node.header.seq = sequence + 1
goal_node.header.stamp = rospy.Time.now()
goal_node.header.frame_id = "world"
goal_node.point = actual_pose.position
mesh_nodes_publisher.publish(goal_node)
rospy.loginfo(goal_node)



with open('/home/user/robot_ws/src/robot_arm_simulate/src/mesh_planned_ordered.txt', 'r') as filehandle:

    filecontents = filehandle.readlines()
    iteration = 0
    for line in filecontents:
        y, z = line[:-1].split("\t")
        y = float(y)
        z = float(z)
        if iteration == 0: desired_position = np.array([[y, z]])
        else : desired_position = np.append(desired_position, [[y, z]], axis=0)
        iteration = iteration + 1

error = np.linalg.norm(real_position - desired_position, axis=1).mean()
rospy.loginfo("Mean error: %f mm" %(1000.0*error))

moveit_commander.roscpp_shutdown()