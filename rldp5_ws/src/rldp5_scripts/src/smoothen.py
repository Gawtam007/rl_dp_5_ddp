#!/usr/bin/env python3

import rospy
import open3d as o3d
import numpy as np
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pcl2

def point_cloud_callback(msg):
    print("Processing point cloud")
    
    # Convert the sensor_msgs/PointCloud2 data to open3d (PointCloud)
    cloud_arr = np.array(list(pcl2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(cloud_arr)

    print('run Poisson surface reconstruction')
    cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30))
    cloud.orient_normals_consistent_tangent_plane(100)
    with o3d.utility.VerbosityContextManager(
            o3d.utility.VerbosityLevel.Debug) as cm:
        mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
            cloud, depth=9)
    
    #pcd = cloud.sample_points_poison_disk(1000)
    #pcd.normals
    
    # Convert the open3d point cloud back to sensor_msgs/PointCloud2
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'world'
    cloud_arr = np.asarray(mesh.points)
    cloud_ros = pcl2.create_cloud_xyz32(header, cloud_arr)

    # Publish the data
    pub.publish(cloud_ros)

rospy.init_node('point_cloud_processing')
sub = rospy.Subscriber("/segmented_point_cloud", PointCloud2, point_cloud_callback)
pub = rospy.Publisher("/processed_point_cloud", PointCloud2, queue_size=1)
rospy.spin()