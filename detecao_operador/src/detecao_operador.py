#!/usr/bin/env python3

import rospy
from augmanity_volumetric_detection.msg import OccupiedPercentage
from octomap_msgs.msg import Octomap


def occupied_percentage_callback(data):

    # print("\nOPERADOR: " + str(data.is_full) + " (" + str(data.percentage) + ")" )
    print(data.resolution)


if __name__ == '__main__':
    # Initialize the node
    rospy.init_node('detecao_operador')

    # # Subscribe to the link states and the point cloud

    # rospy.Subscriber('occupied_percentage', OccupiedPercentage, occupied_percentage_callback)
    rospy.Subscriber('octomap_full', Octomap, occupied_percentage_callback)
    # octomap_pc_callback_partial = partial(octomap_pc_callback, config=config)
    # rospy.Subscriber(octomap_pc_topic, PointCloud2, octomap_pc_callback_partial)
    rospy.spin()
