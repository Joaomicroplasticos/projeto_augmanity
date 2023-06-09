#!/usr/bin/env python3

from augmanity_volumetric_detection.msg import OccupiedPercentage
from colorama import Fore
import rospy
import pyfiglet
from UR10eArm import UR10eArm
from functools import partial
from waiting import wait

operador = False
percentage = []
label = "LIVRE"


def is_operador(operador):
    if not operador:
        return True
    return False

def occupied_percentage_callback(data):
    global percentage
    global label
    global operador
    # label = "LIVRE"
    if len(percentage) < 2:
        percentage.append(data.percentage)
    else:
        percentage[0] = percentage[1]
        percentage[1] = data.percentage
        # percentage[2]
        # percentage[2] = data.percentage
    if len(percentage) == 2:
        # if percentage[0] < percentage[1] < percentage[2] and (percentage[2]-percentage[0]) > 0.1 :
        if percentage[0] < percentage[1] and (percentage[1]-percentage[0]) > 0.02:
            operador = True
            label = "OPERADOR"
        # elif percentage[2] < percentage[1] < percentage[0] and (percentage[0]-percentage[1]) > 0.05:
        elif percentage[1] < percentage[0] and (percentage[0]-percentage[1]) > 0.02:
            operador = False
            label = "LIVRE"

    result = pyfiglet.figlet_format(label, width=500)
    if label == "LIVRE":
        print(Fore.GREEN + result + Fore.RESET)
    elif label == "OPERADOR":
        print(Fore.RED + result + Fore.RESET)

    # if data.is_full:
    #     operador = True
    #     label = "OPERADOR"
    #     result = pyfiglet.figlet_format(label, width=500)
    #     print(Fore.RED + result + Fore.RESET)
    # else:
    #     operador = False
    #     label = "LIVRE"
    #     result = pyfiglet.figlet_format(label, width=500)
    #     print(Fore.GREEN + result + Fore.RESET)


if __name__ == '__main__':
    # Initialize the node
    rospy.init_node('detecao_operador')

    rospy.Subscriber('occupied_percentage', OccupiedPercentage, occupied_percentage_callback)
    arm = UR10eArm()
    # ponto1 = [-0.3445828596698206, -1.1681607824615021, 0.5364492575274866, -0.8584203285029908, -1.5743430296527308,
    #           -0.4578288237201136]
    # ponto2 = [-1.5887554327594202, -2.173347135583395, 1.4872754255877894, -0.841213123207428, -1.6002076307879847,
    #           -1.3123729864703577]
    # ponto3 = [-3.126608673726217, -1.6642056904234828, 1.7471430937396448, -1.678131719628805, -1.6702387968646448,
    #           -2.8480380217181605]

    ponto1 = [0.8888910452472132, -1.9028073749937953, 1.1860785484313965, -0.5265534681132813, -1.53448993364443,
              -0.4198840300189417]

    ponto2 = [1.763186279927389, -2.23720707515859, 1.4745410680770874, -1.925089498559469, -1.4956653753863733,
              -0.0012005011187952164]
    a = 1
    v = 1

    ponto3 = [1.4384377638446253, -2.2871557674803675, 0.4457206130027771, -1.2173187893680115, -1.4388273398028772,
              0.32031264901161194]

    ponto4 = [1.3298237959491175, -1.83217253307485, 0.17638224363327026, -1.3171020907214661, -1.4255626837359827,
              1.7840476036071777]

    ponto5 = [1.5984914938556116, -1.6653062305846156, 0.07236142456531525, -1.6007615528502406, -1.5780790487872522,
              1.9305849075317383]

    ponto6 = [1.8922866026507776, -1.4971657332829018, 0.06581167876720428, -1.9582320652403773, -1.5695379416095179,
              1.9563051462173462]

    ponto7 = [1.515129868184225, -1.8064843616881312, 2.6316964626312256, -1.32545060039077, -1.623082462941305,
              2.2211647033691406]

    ponto8 = [1.8656871954547327, -1.2075888377479096, 2.6145241260528564, -2.243502756158346, -1.6299837271319788,
              2.222665309906006]

    lista_pontos = [ponto1, ponto2, ponto1, ponto3, ponto4, ponto5, ponto6, ponto5, ponto3, ponto1, ponto2, ponto1, ponto3,
                    ponto4, ponto3, ponto1, ponto7, ponto8, ponto7, ponto1]

    rate = rospy.Rate(3000)
    # while not rospy.is_shutdown() and operador==False:

    while not rospy.is_shutdown():
        # if operador:
        #     pass
        # else:
            # for point in [ponto2, ponto1, ponto2, ponto3, ponto2]:
        for point in lista_pontos:
            wait(lambda: is_operador(operador))
            state = arm.go_to_joint_state(point[2], point[1], point[0], point[3], point[4], point[5], v, a)
        rate.sleep()

    rospy.spin()
