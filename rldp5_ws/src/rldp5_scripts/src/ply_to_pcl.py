#!/usr/bin/env python3

import rospy
import pcl
from sensor_msgs.msg import PointCloud2
import pcl_msgs
from std_msgs.msg import Header
import open3d as o3d



def pcl_to_ros(point_cloud_pcl):
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "base_link" 
    point_cloud_ros = PointCloud2()
    point_cloud_ros = pcl.toROSMsg(point_cloud_pcl)
    point_cloud_ros.header = header
    return point_cloud_ros


def ply_to_pcl_and_publish(input_ply_file, output_pcl_file, pcl_pub):

    #cloud = pcl.load(input_ply_file)
    #pcl.save(cloud, output_pcl_file)

    pcd = o3d.io.read_point_cloud("/home/acer/crg/ddp/rldp5_ws/src/rldp5_scripts/src/meshed-poisson.ply")
    o3d.io.write_point_cloud("/home/acer/crg/ddp/rldp5_ws/src/rldp5_scripts/src/file.pcd", pcd)

    pcl_msg = pcl_to_ros(pcd)
    pcl_pub.publish(pcl_msg)
    print("Conversion completed and point cloud published.")


if __name__ == '__main__':
    rospy.init_node('ply_to_pcl')

    input_ply_file = "/home/acer/crg/ddp/rldp5_ws/src/rldp5_scripts/src/meshed-poisson.ply"
    output_pcl_file = "/home/acer/crg/ddp/rldp5_ws/src/rldp5_scripts/src/file.pcl"

    pcl_pub = rospy.Publisher('/point_cloud_topic', PointCloud2, queue_size=10)

    ply_to_pcl_and_publish(input_ply_file, output_pcl_file, pcl_pub)
    