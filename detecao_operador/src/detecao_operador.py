#!/usr/bin/env python3

from augmanity_volumetric_detection.msg import OccupiedPercentage
from colorama import Fore
import rospy
import pyfiglet
from UR10eArm import UR10eArm
from functools import partial

global operador

def occupied_percentage_callback(data):
    if data.is_full:
        operador = True
        label = "OPERADOR"
        result = pyfiglet.figlet_format(label, width=500)
        print(Fore.RED + result + Fore.RESET)
    else:
        operador = False
        label = "LIVRE"
        result = pyfiglet.figlet_format(label, width=500)
        print(Fore.GREEN + result + Fore.RESET)


if __name__ == '__main__':
    # Initialize the node
    rospy.init_node('detecao_operador')
    operador = False

    rospy.Subscriber('occupied_percentage', OccupiedPercentage, occupied_percentage_callback)
    arm = UR10eArm()
    ponto1 = [-0.3445828596698206, -1.1681607824615021, 0.5364492575274866, -0.8584203285029908, -1.5743430296527308,
              -0.4578288237201136]
    ponto2 = [-1.5887554327594202, -2.173347135583395, 1.4872754255877894, -0.841213123207428, -1.6002076307879847,
              -1.3123729864703577]
    ponto3 = [-3.126608673726217, -1.6642056904234828, 1.7471430937396448, -1.678131719628805, -1.6702387968646448,
              -2.8480380217181605]
    a = 1
    v = 1
    rate = rospy.Rate(100)
    # while not rospy.is_shutdown() and operador==False:

    while not rospy.is_shutdown():
        for point in [ponto2, ponto1, ponto2, ponto3, ponto2]:
            if operador:
                pass
            else:
                state = arm.go_to_joint_state(point[0], point[1], point[2], point[3], point[4], point[5], v, a)
        rate.sleep()

    rospy.spin()
