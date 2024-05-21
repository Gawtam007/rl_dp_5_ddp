#!/usr/bin/env python3
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image, PointCloud2, PointField
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
from matplotlib import pyplot as plt
import numpy as np

# Initialize OpenCV bridge
bridge = CvBridge()

# Store detected corners
sorted_points = []

sorted_points = [[-0.5, 100], [100, 0], [0, 100], [100, 100]]



# Callback function for the point cloud subscriber
def point_cloud_callback(msg):
    """
    Callback function for processing point cloud messages.

    Args:
        msg (sensor_msgs.msg.PointCloud2): The input point cloud message.
    """

    print("Message header", msg.header, msg.height, msg.width, msg.point_step, msg.row_step, msg.is_dense)
    '''
    pc_data = list(pc2.read_points(msg))
   
    print(len(pc_data))
    
        i = 0
        points = []
        for data in pc_data:
            i+=1
            x, y, z = data
            points.append([x, y, z])

        print ("Number of points : ",i)
'''
    try:
        segmented_point_cloud = process_point_cloud(msg)

        print("Reurned")
        
        # Publish the segmented point cloud with the corrected header
        publish_segmented_point_cloud(segmented_point_cloud)

    except Exception as e:
        print(e)


def process_point_cloud(points):
    segmented_point_cloud = []
    print(sorted_points)
    try:

        points = np.asarray(list(pc2.read_points(points)))
        print("\n\n", points)

        print("Shape of point", points.shape)

        if sorted_points:
            min_x = 0.355
            max_x = 0.67
            min_y = -0.2
            max_y = 0.2

            print(f"min_x: {min_x}\nmax_x: {max_x}\nmin_y: {min_y}\nmax_y: {max_y}")

            segmented_point_cloud = [point for point in points if min_x <= point[0] <= max_x and min_y <= point[1] <= max_y]

            print ("Shape of segment", len(segmented_point_cloud))
            
            return segmented_point_cloud
        else:
            print("Sorted points are not available.")
            return []

    except Exception as e:
        print("Error in process_point_cloud:", e)
        return []

def publish_segmented_point_cloud(segmented_point_cloud):
    # Create a PointCloud2 message for the segmented point cloud
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'world'
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=32, datatype=PointField.FLOAT32, count=1),
        PointField(name="normal_x", offset=16, datatype=PointField.FLOAT32, count=1),
        PointField(name="normal_y", offset=20, datatype=PointField.FLOAT32, count=1),
        PointField(name="normal_z", offset=24, datatype=PointField.FLOAT32, count=1),
        PointField(name="curvature", offset=36, datatype=PointField.FLOAT32, count=1),
    ]

    cloud_msg = pc2.create_cloud(header, fields, segmented_point_cloud)
    pub_segmented_point_cloud.publish(cloud_msg)

if __name__ == "__main__":
    # Initialize ROS node
    rospy.init_node("point_cloud_segmentation")

    # Subscribe to the point cloud topic
    point_cloud_topic = "/point_cloud"
    point_cloud_sub = rospy.Subscriber(point_cloud_topic, PointCloud2, point_cloud_callback)

    # Create a ROS publisher for the segmented point cloud
    pub_segmented_point_cloud = rospy.Publisher("/segmented_point_cloud", PointCloud2, queue_size=10)

    # Spin ROS node
    rospy.spin()
