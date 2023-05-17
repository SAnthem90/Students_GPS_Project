#!/usr/bin/env python
# import the opencv library
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
# define a video capture object
bridge = CvBridge()
vid = cv2.VideoCapture(0)
image_pub = rospy.Publisher("/video_topic", Image, queue_size=10)
rospy.init_node('video_publisher')

while not rospy.is_shutdown():

   ret, frame = vid.read()
   ros_image = bridge.cv2_to_imgmsg(frame, "bgr8")
   image_pub.publish(ros_image)
# After the loop release the cap object
vid.release()
# Destroy all the windows
cv2.destroyAllWindows()
