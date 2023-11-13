#!/usr/bin/env python
import math
from math import pi
import rospy
import tf
from robot_arm_simulate.srv import ik, ikResponse


def callback(request):

    q1 = 0.0
    q2 = 0.0
    q3 = 0.0

    L0 = request.L0
    L1 = request.L1
    L2 = request.L2
    L3 = request.L3

    y0 = 0
    z0 = L0 + L1 + L2 + L3

    joint_values = []
    positions_aux = []

    with open('/home/user/robot_ws/src/robot_arm_simulate/src/mesh_planned.txt', 'r') as filehandle:
        filecontents = filehandle.readlines()

        for line in filecontents:
            xx, yy, zz = line[:-1].split("\t")
            y = float(yy)
            z = float(zz)
            positions_aux.append([y, z])

        i = 0
        index = i
        dist_min = 1000000.0
        for pos in positions_aux:
            aux = math.sqrt((pos[0]-y0)**2 + (pos[1]-z0)**2)
            if(aux < dist_min):
                dist_min = aux
                index = i
            i = i + 1

        if (index == 0):
            positions = positions_aux
        else:
            positions = positions_aux[index:] + positions_aux[:index]

    with open('/home/user/robot_ws/src/robot_arm_simulate/src/mesh_planned_ordered.txt', 'w') as my_file:

        for pos in positions:
            my_file.write(str(pos[0]) + "\t" + str(pos[1]) + "\n")
            y = pos[0]
            z = pos[1]
            z1 = z - L3
            z2 = z1 - L0

            q2 = -math.acos((z2**2 + y**2 - L1**2 - L2**2)/(2*L1*L2))
            q1 = math.atan2(z2, y) - math.atan2(L2*math.sin(q2),
                                                L1 + L2*math.cos(q2)) - pi/2   
            q3 = -(q1 + q2)

            joint_values.append([q1, q2, q3])

    with open('/home/user/robot_ws/src/robot_arm_simulate/src/joint_values.txt', 'w') as my_file:
        for fila in joint_values:
            my_file.write(str(fila[0]) + "\t" +
                          str(fila[1]) + "\t" + str(fila[2]) + "\n")

    return len(joint_values)


if __name__ == '__main__':

    rospy.init_node('inverse_kinematics')
    listener = tf.TransformListener()
    service = rospy.Service('ik_algorithm', ik, callback)
    rospy.spin()
