#!/usr/bin/env python

#chmod +x robot_vision.py

# Python dependencies
import numpy as np
import cv2
import math
from cv_bridge import CvBridge, CvBridgeError

# ROS dependencies
import rospy
from sensor_msgs.msg import Image, LaserScan
from visualization_msgs.msg import Marker

SCAN_SIZE = 902
bridge = CvBridge()
pub = rospy.Publisher('visualization_marker', Marker, queue_size=100)
scan_data = np.zeros((SCAN_SIZE, ), dtype=np.float32)

def rviz_mark(face):
    marker = Marker()
    marker.header.frame_id = "laser_link"
    marker.type = marker.SPHERE # marker.TEXT_VIEW_FACING
    marker.action = marker.ADD
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.pose.orientation.w = 1.0
    marker.id = 0

    '''
    x, y, w, h = face
    im_pix_h = h / 2
    im_theta = ((math.pi / 8) * h) / (512)
    dist_pix = (im_pix_h) / math.tan(im_theta) / 3*2
    dist_x_real = ((dist_pix * 0.5) / im_pix_h)

    marker.pose.position.x = dist_x_real
    marker.pose.position.y = ((x + w / 2) / 512)
    '''

    x_offset = x + w // 2 - 256
    angle = x_offset / 256.0 * (math.pi / 8)
    scan_id = int((angle + math.pi) * SCAN_SIZE / math.pi / 2)
    scan_dist = scan_data[scan_id]
    marker.pose.position.x = math.sin(angle) * scan_dist
    marker.pose.position.y = -math.cos(angle) * scan_dist
    marker.pose.position.z = 0
    pub.publish(marker)

def scan_callback(msg):
    scan_data[:] = msg.ranges

def image_callback(msg):
    cv2_img = None
    try:
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)

    #hsv_img = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2HSV)

    #cv2.imshow('cv2', cv2_img)
    #cv2.waitKey(1)
    #cv2.imshow('hsv', hsv_img)
    #cv2.waitKey(1)

    face_cascade = cv2.CascadeClassifier('/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml')
    # Convert into grayscale
    gray_img = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2GRAY)
    # Detect faces
    faces = face_cascade.detectMultiScale(gray_img, 2, 4)
    # Draw rectangle around the faces
    for (x, y, w, h) in faces:
    	cv2.rectangle(cv2_img, (x, y), (x+w, y+h), (255, 0, 0), 2)
        rviz_mark((x, y, w, h))
    # Display the output
    cv2.imshow('marked', cv2.flip(cv2_img, 1))
    cv2.waitKey(1)


def main():
    rospy.init_node('robot_vision')
    image_topic = ""
    rospy.Subscriber('/vrep/image', Image, image_callback)
    rospy.Subscriber('/vrep/scan', LaserScan, scan_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
