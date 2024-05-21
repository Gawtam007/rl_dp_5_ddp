#!/usr/bin/env python3

import open3d as o3d
import numpy as np
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pcl2
import rospy

def point_cloud_callback(msg):
    # Convert the sensor_msgs/PointCloud2 data to pcl (PointCloud)
    cloud = pcl2.read_points(msg, field_names=("x", "y", "z"))
    cloud_arr = np.array(list(cloud), dtype=np.float32)
    cloud_o3d = o3d.geometry.PointCloud()
    cloud_o3d.points = o3d.utility.Vector3dVector(cloud_arr)

    # Smoothen the point cloud using a moving least squares
    radius = 0.001
    max_nn = 10
    cloud_o3d.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=max_nn))
    cloud_smoothed = o3d.geometry.PointCloud()
    cloud_smoothed.points = o3d.utility.Vector3dVector(np.asarray(cloud_o3d.normals))

    # Convert the open3d point cloud back to sensor_msgs/PointCloud2
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'world'
    cloud_ros = pcl2.create_cloud_xyz32(header, np.asarray(cloud_smoothed.points))

    # Publish the data
    pub.publish(cloud_ros)

rospy.init_node('point_cloud_processing')
sub = rospy.Subscriber("/segmented_point_cloud", PointCloud2, point_cloud_callback)
pub = rospy.Publisher("/processed_point_cloud", PointCloud2, queue_size=1)
rospy.spin()