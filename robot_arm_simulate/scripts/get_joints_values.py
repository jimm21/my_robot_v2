#!/usr/bin/env python
import rospy
from robot_arm_simulate.srv import parameters, parametersResponse
from robot_arm_simulate.srv import sp, spResponse
from robot_arm_simulate.srv import ik, ikResponse

def tfParameters():
    global tf_values
    global got_it
    got_it = False
    rospy.wait_for_service("tf_parameters")
    rate = rospy.Rate(1)

    while not (rospy.is_shutdown() or got_it):
        try:
            get_tf_parameters = rospy.ServiceProxy("tf_parameters", parameters)
            tf_values = get_tf_parameters()
            got_it = True
            print("tf", tf_values)

        except rospy.ServiceException:
            continue

        rate.sleep()

def sequencePlanning():
    global sp_distance
    global got_it
    got_it = False
    rospy.wait_for_service("sp_algorithm")
    rate = rospy.Rate(1)

    while not (rospy.is_shutdown() or got_it):
        try:
            do_sequence_planning = rospy.ServiceProxy("sp_algorithm", sp)
            sp_distance = do_sequence_planning()
            got_it = True
            print("sp", sp_distance)

        except rospy.ServiceException:
            continue

        rate.sleep()

def inverseKinematic():
    global ik_cnt
    global got_it
    got_it = False
    rospy.wait_for_service("ik_algorithm")
    rate = rospy.Rate(1)

    while not (rospy.is_shutdown() or got_it):
        try:
            do_inverse_kinematics = rospy.ServiceProxy("ik_algorithm", ik)
            ik_cnt = do_inverse_kinematics(tf_values.L0, tf_values.L1, tf_values.L2, tf_values.L3)
            got_it = True
            print("ik", ik_cnt)

        except rospy.ServiceException:
            continue

        rate.sleep()

if __name__ == '__main__':
    rospy.init_node("get_joints_values")

    got_it = False
    tf_values = parametersResponse()
    sp_distance = spResponse()
    ik_cnt = ikResponse()

    tfParameters()
    sequencePlanning()
    inverseKinematic()
