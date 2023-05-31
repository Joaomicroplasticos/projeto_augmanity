#!/usr/bin/env python3
import random
import tf
import rospy
import numpy as np
import ros_numpy
from scipy.spatial.transform import Rotation as R
from sensor_msgs import point_cloud2 as pc2
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
from augmanity_volumetric_detection.msg import OccupiedPercentage

class DetectedCell:
    def __init__(self, side_lengths, world_frame, position, threshold, visualize=False):
        self.side_lengths = side_lengths
        self.world_frame = world_frame
        self.position = position
        self.marker_color = [random.random() for _ in range(3)]
        self.vertices = self.compute_vertices()
        self.total_volume = np.prod(self.side_lengths)
        self.percentage_pub = rospy.Publisher('occupied_percentage', OccupiedPercentage, queue_size=10)
        self.threshold = threshold
        self.visualize = visualize
        self.marker = self.create_marker_rect_prism()
        self.occupied_percentage_msg = self.create_occupied_percentage_msg()       
        if self.visualize:
            self.cell_pc_pub = rospy.Publisher('occupied_voxels_in_cell', PointCloud2, queue_size=10)
            self.marker_pub = rospy.Publisher('cell_volume_marker', Marker, queue_size=10)


    def is_inside(self, pointcloud_msg, listener, resolution):
        """Verify which voxels are inside the predefined volume, calculate the percentage of that volume 
        filled with voxels

        Args:
            pointcloud_msg (PointCloud2): message with the centers of the occupied voxels given by octomap
            listener (tf2 listener): ros tf2 listener
            resolution (float32): resolution of the octomap
        """
        # Convert the pointcloud message to a numpy array
        pointcloud = np.array(list(pc2.read_points(pointcloud_msg, skip_nans=True, field_names=("x", "y", "z"))))

        if pointcloud_msg.header.frame_id != self.world_frame:
            # Transform the pointcloud to the world frame
            pointcloud = np.transpose(self.transform(pointcloud_msg.header.frame_id, listener, pointcloud))

        # Compute the points inside the prism
        inside = np.array([point for point in pointcloud
                           if self.is_inside_prism(point, self.vertices)])

        # Calculate the volume of the voxels inside the cell and what percentage is it of the total cell
        voxel_volume = inside.shape[0] * resolution ** 3
        percentage = voxel_volume / self.total_volume
        
        if percentage >= self.threshold:
            self.occupied_percentage_msg.is_full = True
        else:
            self.occupied_percentage_msg.is_full = False

        # Convert the array to PointCloud2
        inside_pointcloud = self.numpy2pc2(inside, pointcloud_msg)

        # Refresh and publish the occupied percentage message
        self.occupied_percentage_msg.header = pointcloud_msg.header
        self.occupied_percentage_msg.percentage = percentage
        self.percentage_pub.publish(self.occupied_percentage_msg)


        # Visualize if requested
        if self.visualize:
            self.cell_pc_pub.publish(inside_pointcloud)
            self.marker_pub.publish(self.marker)

        
    def compute_vertices(self):
        """Compute the vertices of the prism 

        Returns:
            vertices (np.array): numpy array with the 8 vertices of the prism
        """
        
        vertices = np.array([[self.position[0] - self.side_lengths[0]/2, self.position[1] - self.side_lengths[1]/2, self.position[2]],
                             [self.position[0] + self.side_lengths[0]/2, self.position[1] - self.side_lengths[1]/2, self.position[2]],
                             [self.position[0] + self.side_lengths[0]/2, self.position[1] + self.side_lengths[1]/2, self.position[2]],
                             [self.position[0] - self.side_lengths[0]/2, self.position[1] + self.side_lengths[1]/2, self.position[2]],
                             [self.position[0] - self.side_lengths[0]/2, self.position[1] - self.side_lengths[1]/2, self.position[2] + self.side_lengths[2]],
                             [self.position[0] + self.side_lengths[0]/2, self.position[1] - self.side_lengths[1]/2, self.position[2] + self.side_lengths[2]],
                             [self.position[0] + self.side_lengths[0]/2, self.position[1] + self.side_lengths[1]/2, self.position[2] + self.side_lengths[2]],
                             [self.position[0] - self.side_lengths[0]/2, self.position[1] + self.side_lengths[1]/2, self.position[2] + self.side_lengths[2]]
        ])


        return vertices

    
    def create_occupied_percentage_msg(self):
        """Create the occupied percentage message, using some immutable values

        Returns:
            OccupiedPercentage: occupied percentage message with immutable values placed
        """
        occupied_percentage_msg = OccupiedPercentage()
        occupied_percentage_msg.side_lengths = list(self.side_lengths)
        occupied_percentage_msg.position = list(self.position)
        occupied_percentage_msg.is_full = False
        return occupied_percentage_msg



    def transform(self, sensor_frame, listener, points):
        """Transform points to the world frame

        Args:
            sensor_frame (string): sensor frame in ROS
            listener (tf.listener): listener of ROS tfs
            points (np.array): pointcloud

        Returns:
            transformed_points (np.array): Pointcloud transformed to the world frame
        """

        # Retrieve transform between world and sensor frame and process it into rotation matrix and translation vector
        try:
            (trans_sensor2world,quarterion_sensor2world) = listener.lookupTransform(self.world_frame, sensor_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException):
            return
        rot_sensor2world = R.from_quat(quarterion_sensor2world)
        R_matrix_sensor2world = rot_sensor2world.as_matrix()
        T_vector_sensor2world = np.array([[trans_sensor2world[0]], [trans_sensor2world[1]], [trans_sensor2world[2]]])

        # Transform prism vertices
        transformed_points = np.matmul(R_matrix_sensor2world, np.transpose(points)) + T_vector_sensor2world
        
        return transformed_points


    def is_inside_prism(self, point, prism_vertices):
        """Verifies if the point is inside the prism by checking if the point is on the inside of the 6 faces

        Args:
            point (np.array): point to verify if it is inside the prism
            prism_vertices (np.array): vertices of the prism

        Returns:
            (bool): Returns True if the point is inside the cylinder
        """

        face_normals = self.calculate_face_normals(prism_vertices)

        for i in range(6):
            # Determine the plane equation for the ith face
            a, b, c = face_normals[i]
            d = -np.dot(face_normals[i], prism_vertices[i])

            # Check if the point is on the inside of the ith face
            if a*point[0] + b*point[1] + c*point[2] + d < 0:
                return False

        return True


    def calculate_face_normals(self, prism_vertices):
        """Function to calculate the normal vector of each face

        Args:
            prism_vertices (np.array): vertices of a prism

        Returns:
            face_normals (np.array): array with the normal vector of each face
        """
        # Define the vertices of each face
        faces = [
            (0, 1, 2, 3),
            (4, 5, 6, 7),
            (0, 1, 5, 4),
            (1, 2, 6, 5),
            (2, 3, 7, 6),
            (3, 0, 4, 7)
        ]

        # Calculate the normal vector of each face
        face_normals = []
        for face in faces:
            v1 = np.array(prism_vertices[face[0]])
            v2 = np.array(prism_vertices[face[1]])
            v3 = np.array(prism_vertices[face[2]])
            face_normal = np.cross(v2 - v1, v3 - v1)
            face_normals.append(tuple(face_normal))

        return face_normals

        
    def create_marker_rect_prism(self):
        """Create a marker for every prism vertex

        Args:
            index (int): specific index for each joint
            vertices (np.array): array with the coordinates of every vertex

        Returns:
            (Marker): Marker with every vertex
        """
        # Create a Line Strip marker
        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.MODIFY

        marker.pose.position.x = self.position[0]
        marker.pose.position.y = self.position[1]
        marker.pose.position.z = self.position[2] + self.side_lengths[2]/2

        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1

        marker.scale.x = self.side_lengths[0]
        marker.scale.y = self.side_lengths[1]
        marker.scale.z = self.side_lengths[2]

        marker.color.r = self.marker_color[0]
        marker.color.g = self.marker_color[1]
        marker.color.b = self.marker_color[2]
        marker.color.a = 0.5

        marker.points.append(Point(0, 0, 0))

        return marker

    
    def numpy2pc2(self, numpy_array, original_msg):

        pc_array = np.zeros(len(numpy_array), dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('intensity', np.float32),
        ])
        pc_array['x'] = numpy_array[:, 0]
        pc_array['y'] = numpy_array[:, 1]
        pc_array['z'] = numpy_array[:, 2]
        pc_array['intensity'] = 1

        pc2_msg = ros_numpy.msgify(PointCloud2, pc_array, stamp=original_msg.header.stamp, 
                                   frame_id=original_msg.header.frame_id)
        return pc2_msg
    