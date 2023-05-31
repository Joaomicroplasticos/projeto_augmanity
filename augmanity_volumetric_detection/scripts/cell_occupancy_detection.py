#!/usr/bin/env python3

from functools import partial
from cell_volume_class import DetectedCell
import numpy as np
import rospy
import tf
from octomap_msgs.msg import Octomap
from sensor_msgs.msg import PointCloud2


def octomap_callback(msg, config):
    """Gather octomap message

    Args:
        msg (octomap): message with octomap
        config (dict): mutable variable to store the message
    """
    config['octomap_msg'] = msg
    # config['resolution'] = msg.resolution
    config['resolution'] = 10
    return

def octomap_pc_callback(msg, config):
    """Gather a pointcloud, which points are the centers of the occupied voxels defined by octomap 

    Args:
        msg (PointCloud2): Pointcloud message with the centers of the occupied voxels
        config (dict): mutable variable to store the message
    """
    config['octomap_pc_msg'] = msg
    return




def main():
    # Define initial parameters
    world_frame = 'camera_depth_optical_frame' #Referencial da camâra
    octomap_topic = '/octomap_full'
    octomap_pc_topic = '/octomap_point_cloud_centers'
    position = np.array([0, -0.5, 1.5]) #definição da posição do volume em relação a câmara
    side_lengths = np.array([2, 1.5, 1.5])#definição do cumprimento de cada lado do paralelopipedo
    threshold = 0.05
    visualize = True
    config = dict()
    config['octomap_msg'] = None
    config['resolution'] = None
    config['octomap_pc_msg'] = None
    config['cell_volume'] = None


    # Initialize the node
    rospy.init_node('cell_occupancy_detection')

    # Subscribe to the link states and the point cloud
    octomap_callback_partial = partial(octomap_callback, config=config)
    rospy.Subscriber(octomap_topic, Octomap, octomap_callback_partial)
    octomap_pc_callback_partial = partial(octomap_pc_callback, config=config)
    rospy.Subscriber(octomap_pc_topic, PointCloud2, octomap_pc_callback_partial)

    # Defining the cell volume
    config['cell_volume'] = DetectedCell(side_lengths, world_frame, position, threshold, visualize)
    
    # Defining rate
    rate = rospy.Rate(30)

    # Defining listener
    listener = tf.TransformListener()

    # Run
    while not rospy.is_shutdown():
        if config['resolution'] is None or config['octomap_pc_msg'] is None:
            continue

        config['cell_volume'].is_inside(config['octomap_pc_msg'], listener, config['resolution'])

        rate.sleep()


if __name__ == '__main__':
    main()
