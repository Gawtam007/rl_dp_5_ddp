#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge 
import cv2 
import os

def callback(data):

  br = CvBridge()
  rospy.loginfo("receiving video frame")
   
  # Convert ROS Image message to OpenCV image
  current_frame = br.imgmsg_to_cv2(data)
   
  # Display image
  cv2.imshow("camera", current_frame)
  print("image recieved")
  try:
  	image_filename = os.path.join("/home/acer/crg/ddp/rldp5_ws/src/rldp5_scripts/src/test_frames", 'image_{}.jpg'.format(rospy.Time.now()))
  	cv2.imwrite(image_filename, current_frame)
  	rospy.loginfo("Saved image as {}".format(image_filename))
  except Exception as e:
        rospy.logerr("Error processing image: {}".format(e))

  cv2.waitKey(400)
      
def receive_message():
  print("sad")
  rospy.init_node('video_sub_py', anonymous=True)
  rospy.Subscriber('image_raw', Image, callback)
  rospy.spin()
  cv2.destroyAllWindows()
  
if __name__ == '__main__':
  receive_message()
