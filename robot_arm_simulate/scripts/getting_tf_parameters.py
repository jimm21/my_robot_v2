#!/usr/bin/env python  
import rospy
import tf
from robot_arm_simulate.srv import parameters, parametersResponse
#import roslib
#roslib.load_manifest('urdf_manipulator_simulate')

def callback(request):

    got_it = False
    rate = rospy.Rate(1)

    while not (rospy.is_shutdown() or got_it):
        try:
            (trans_bl,rot_bl) = listener.lookupTransform('/world', '/base_link', rospy.Time(0))
            #print("trans_bl", trans_bl)
            #print("rot_bl", rot_bl)
            d0 = trans_bl[2]
            got_it = True

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()

    got_it = False
    
    while not (rospy.is_shutdown() or got_it):
        try:
            (trans_Fl,rot_Fl) = listener.lookupTransform('/base_link', '/First_link', rospy.Time(0))
            #print("trans_Fl", trans_Fl)
            #print("rot_Fl", rot_Fl)
            d1 = trans_Fl[2]
            a1 = trans_Fl[0]
            got_it = True

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()

    got_it = False

    while not (rospy.is_shutdown() or got_it):
        try:
            (trans_Sl,rot_Sl) = listener.lookupTransform('/First_link', '/Second_link', rospy.Time(0))
            #print("trans_Sl", trans_Sl)
            #print("rot_Sl", rot_Sl)
            d2 = trans_Sl[2]
            a2 = trans_Sl[0]
            got_it = True

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()

    got_it = False

    while not (rospy.is_shutdown() or got_it):
        try:
            (trans_Tl,rot_Tl) = listener.lookupTransform('/Second_link', '/Third_link', rospy.Time(0))
            #print("trans_Tl", trans_Tl)
            #print("rot_Tl", rot_Tl)
            d3 = trans_Tl[2]
            a3 = trans_Tl[0]
            got_it = True

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()

    got_it = False

    while not (rospy.is_shutdown() or got_it):
        try:
            (trans_Ee,rot_Ee) = listener.lookupTransform('/Third_link', '/End_effector', rospy.Time(0))
            #print("trans_Ee", trans_Ee)
            #print("rot_Ee", rot_Ee)
            de = trans_Ee[2]
            ae = trans_Ee[0]
            got_it = True

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()

    L0 = d0 + a1
    L1 = a2
    L2 = a3
    L3 = ae

    return parametersResponse(L0, L1, L2, L3)

if __name__ == '__main__':

    rospy.init_node('tf_parameters_service')
    listener = tf.TransformListener()
    service = rospy.Service('tf_parameters', parameters, callback)
    rospy.spin()