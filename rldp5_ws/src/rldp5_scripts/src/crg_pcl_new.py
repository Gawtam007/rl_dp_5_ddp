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

def visualize_normals(normal_vectors, point_array):
    """
    Visualizes normals as markers in Rviz.

    Args:
        normal_vectors (numpy.ndarray): Array of tangent vectors representing normals.
        point_array (numpy.ndarray): Array of 3D points corresponding to the point cloud.

    Returns:
        None

    Frame:
        Setting the frame to royale_camera_0_optical_frame is important because, 
        similar to our robot's TF, royale_camera_0_optical_frame is base_link and royale_camera_link is the world frame
    """
    marker_array = MarkerArray()

    for i in range(0, len(point_array), 40):
            normal_marker = Marker()
            normal_marker.header.frame_id = "world"
            normal_marker.header.stamp = rospy.Time.now()
            normal_marker.id = i
            normal_marker.type = Marker.ARROW
            normal_marker.action = Marker.ADD

            position_np = point_array[i]
            normal_np = normal_vectors[i]

            normal_marker.pose.position.x = position_np[0]
            normal_marker.pose.position.y = position_np[1]
            normal_marker.pose.position.z = position_np[2]

            # print(f"Marker_{i} position: \n{normal_marker.pose.position}")

            # Check for zero-length tangent vectors before normalization
            if np.linalg.norm(normal_np) != 0:
                normal_np /= np.linalg.norm(normal_np)

                z_axis_local = 1 * normal_np
                y_axis_global = np.array([0, 1, 0]).astype(float)
                x_axis_local = np.cross(z_axis_local, y_axis_global).astype(float)
                y_axis_local = np.cross(z_axis_local, x_axis_local).astype(float)

                x_axis_local /= np.linalg.norm(x_axis_local)
                y_axis_local /= np.linalg.norm(y_axis_local)
                z_axis_local /= np.linalg.norm(z_axis_local)

                rotation_matrix = np.column_stack((x_axis_local, y_axis_local, z_axis_local))
                quaternion_data = quaternion_from_matrix(np.vstack([np.hstack([rotation_matrix, np.zeros((3, 1))]), [0, 0, 0, 1]]))

                quaternion_data /= np.linalg.norm(quaternion_data)

                normal_marker.pose.orientation.x = quaternion_data[0]
                normal_marker.pose.orientation.y = quaternion_data[1]
                normal_marker.pose.orientation.z = quaternion_data[2]
                normal_marker.pose.orientation.w = quaternion_data[3]
            else:
                # If the tangent vector is zero, set an arbitrary orientation
                normal_marker.pose.orientation.w = 1.0

            normal_marker.scale.x = 0.01
            normal_marker.scale.y = 0.001
            normal_marker.scale.z = 0.001
            normal_marker.color.a = 1.0
            normal_marker.color.r = 1.0 
            normal_marker.color.g = 1.0 
            normal_marker.color.b = 0.0 

            marker_array.markers.append(normal_marker)

    marker_pub.publish(marker_array)

    print("published markers")


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
    o3d_cloud.orient_normals_consistent_tangent_plane(100)
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
    points = np.asarray(list(pc2.read_points(cloud_msg)))

    print(f"Shape of point cloud: {points.shape}")
    # print(f"Shape of point cloud: {.point_cloud.shape}")
    print ("p 1")

    if len(points.shape) == 1:
            print("Reshaping the Point Cloud np array")
            points = points.reshape(-1, 3)

    normal_vectors = compute_normals(points)

    print ("p 2")
    visualize_normals(normal_vectors, points)

    waypoints = []

    for i, normal in enumerate(normal_vectors):
            z_axis_local = 1 * normal
            y_axis_global = np.array([0, 1, 0]).astype(float)
            x_axis_local = np.cross(z_axis_local, y_axis_global).astype(float)
            x_axis_local /= np.linalg.norm(x_axis_local)
            y_axis_local = np.cross(z_axis_local, x_axis_local).astype(float)
            y_axis_local /= np.linalg.norm(y_axis_local)

            rotation_matrix = np.column_stack((x_axis_local, y_axis_local, z_axis_local))
            quaternion_data = quaternion_from_matrix(np.vstack([np.hstack([rotation_matrix, np.zeros((3, 1))]), [0, 0, 0, 1]]))

            quaternion_data /= np.linalg.norm(quaternion_data)

            pose_goal = Pose()
            pose_goal.position.x = points[i][0]
            pose_goal.position.y = points[i][1]
            pose_goal.position.z = points[i][2]
            pose_goal.orientation.x = quaternion_data[0]
            pose_goal.orientation.y = quaternion_data[1]
            pose_goal.orientation.z = quaternion_data[2]
            pose_goal.orientation.w = quaternion_data[3]

            # print(f"Type of pose_goal: {type(pose_goal)}")
            waypoints.append(pose_goal)

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