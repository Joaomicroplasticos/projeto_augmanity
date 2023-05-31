#! /usr/bin/python3

from UR10eArm import UR10eArm
import rospy

if __name__ == '__main__':
    rospy.init_node("move_robot_test", anonymous=True)
    # --------------------------------------------------------------------
    # -------------------------initialization-----------------------------
    # --------------------------------------------------------------------
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
    while not rospy.is_shutdown():
        state = arm.go_to_joint_state(ponto2[0], ponto2[1], ponto2[2], ponto2[3], ponto2[4], ponto2[5], v, a)
        state = arm.go_to_joint_state(ponto1[0], ponto1[1], ponto1[2], ponto1[3], ponto1[4], ponto1[5], v, a)
        state = arm.go_to_joint_state(ponto2[0], ponto2[1], ponto2[2], ponto2[3], ponto2[4], ponto2[5], v, a)
        state = arm.go_to_joint_state(ponto3[0], ponto3[1], ponto3[2], ponto3[3], ponto3[4], ponto3[5], v, a)
        state = arm.go_to_joint_state(ponto2[0], ponto2[1], ponto2[2], ponto2[3], ponto2[4], ponto2[5], v, a)

        rate.sleep()
