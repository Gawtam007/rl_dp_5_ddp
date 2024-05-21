#!/usr/bin/env python3

from geometry_msgs.msg import Pose, TransformStamped
from sensor_msgs.msg import JointState, PointCloud2
from sklearn.neighbors import NearestNeighbors
from std_msgs.msg import String
from tf.transformations import quaternion_from_matrix, quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray

import moveit_commander
import moveit_msgs.msg
import numpy as np
import open3d as o3d
import rospy
import sensor_msgs.point_cloud2 as pc2
import sys
import tf2_ros

try:
    from math import pi
except ImportError:
    pi = 3.141592653589793

# Initialize ROS node
rospy.init_node('visualize_normals_and_move_robot', anonymous=True)

'''
# Initialize MoveIt
moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "rldp5_arm"
arm_group = moveit_commander.MoveGroupCommander(group_name)

rospy.sleep(1) 
'''

# Publishers   
global marker_pub, display_trajectory_pub

marker_pub = rospy.Publisher('/normals_markers', MarkerArray, queue_size=10)

display_trajectory_pub = rospy.Publisher(
    "/move_group/display_planned_path",
    moveit_msgs.msg.DisplayTrajectory,
    queue_size=20,
)

def visualize_normals(points, normals):
    """
    Visualizes normals as arrows in Rviz and transforms them into poses.

    Args:
        points (numpy.ndarray): Array of 3D points.
        normals (numpy.ndarray): Array of corresponding normal vectors.

    Returns:
        list: List of poses generated from the normals.
    """

    waypoints_list = []

    if points is not None and normals is not None:
        min_length = min(len(points), len(normals))
        
        marker_array = MarkerArray()

        for i in range(min_length):
            point = points[i]
            normal = normals[i]
            quaternion = quaternion_from_normal(normal)

            pose_goal = Pose()
            pose_goal.position.x = point[0]
            pose_goal.position.y = point[1]
            pose_goal.position.z = point[2]
            pose_goal.orientation.x = quaternion[0]
            pose_goal.orientation.y = quaternion[1]
            pose_goal.orientation.z = quaternion[2]
            pose_goal.orientation.w = quaternion[3]

            waypoints_list.append(pose_goal)

            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "normals"
            marker.id = i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            
            marker.pose.position.x = point[0]
            marker.pose.position.y = point[1]
            marker.pose.position.z = point[2]

            marker.pose.orientation.x = quaternion[0]
            marker.pose.orientation.y = quaternion[1]
            marker.pose.orientation.z = quaternion[2]
            marker.pose.orientation.w = quaternion[3]
            # marker.pose.orientation.w = -1.0

            marker.scale.x = 0.01  # Arrow shaft diameter
            marker.scale.y = 0.001  # Arrow head diameter
            marker.scale.z = 0.001  # Arrow head length
            marker.color.a = 1.0  # Alpha (transparency)
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0

            marker_array.markers.append(marker)

        marker_pub.publish(marker_array)
        print("Published the markers of normals")

    return waypoints_list

def quaternion_from_normal(normal):
    """
    Computes quaternion from a given normal vector.

    Args:
        normal (numpy.ndarray): Normal vector.

    Returns:
        numpy.ndarray: Quaternion representing the orientation.
    """
    # Using Gram-Schmidt Process
    y_axis_global = [0.0, 1.0, 0.0]
    z_axis_local = 1*normal
    x_axis_local = np.cross(normal, y_axis_global)
    x_axis_local/= np.linalg.norm(x_axis_local)
    y_axis_local= np.cross(z_axis_local,x_axis_local)
    y_axis_local/= np.linalg.norm(y_axis_local)
    z_axis_local/= np.linalg.norm(z_axis_local)
    rotation_matrix = np.column_stack((x_axis_local,y_axis_global,z_axis_local))
    quaternion = quaternion_from_matrix(np.vstack([np.hstack([rotation_matrix, np.zeros((3, 1))]), [0, 0, 0, 1]]))

    quaternion /= np.linalg.norm(quaternion)
  #  print(f"Quaternion: {quaternion}")
    return quaternion

def compute_normals(point_cloud_msg, k_neighbors=30):
    """
    Computes normals for a point cloud using Open3D library.

    Args:
        point_cloud (numpy.ndarray): Array of 3D points.
        k_neighbors (int): Number of neighbors for normal estimation.

    Returns:
        numpy.ndarray: Array of normal vectors.
    
    Methods:
        1. Using Open3D module
        2. Normal Estimation from PCL (Check other codes probably testcode.cpp & move_robot.py) for reference
    """

    #print("1")

    # Extract x, y, z coordinates
    point_cloud = point_cloud_msg[:, :3]
    print(point_cloud.shape)
    # Convert point cloud to Open3D format
    o3d_cloud = o3d.geometry.PointCloud()
    o3d_cloud.points = o3d.utility.Vector3dVector(point_cloud)

    # Compute normals using Open3D (CUDA version)
    o3d_cloud.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamKNN(knn=k_neighbors),
        fast_normal_computation=True  # Enable CUDA acceleration
    )

    # Get the normals as a numpy array
    normals = np.asarray(o3d_cloud.normals)
    
    return normals

def pc_callback(cloud_msg):
    """
    Callback function for processing point cloud messages.

    Args:
        cloud_msg (sensor_msgs.msg.PointCloud2): Point cloud message.
    """
   
    #point_cloud = np.asarray(list(pc2.read_points(cloud_msg, skip_nans=True)))
    point_cloud = np.asarray(list(pc2.read_points(cloud_msg)))

    print(f"Shape of point cloud: {point_cloud.shape}")
    # print(f"Shape of point cloud: {.point_cloud.shape}")
    print ("p 1")

    computed_normals = compute_normals(point_cloud)

    print ("p 2")
    waypoints = visualize_normals(point_cloud, computed_normals)

    print ("p 3")
    print("pose onbject 1 :",waypoints[0])
    print(f"Shape of waypoints: {np.asarray(waypoints).shape}")

def listener():

    tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rospy.Subscriber('/segmented_point_cloud', PointCloud2, pc_callback)
    # rospy.Subscriber('/cloud_normals_something', PointCloud2, data_processor.nor_callback)


    rospy.spin()

if __name__ == '__main__':
    listener()