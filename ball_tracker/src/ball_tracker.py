#! /usr/bin/python

# Python dependencies
import numpy as np
import cv2
from simple_pid import PID
from cv_bridge import CvBridge, CvBridgeError

# ROS dependencies
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

bridge = CvBridge()
CONTOUR_MIN_POINTS = 10
DIST_CONST = 1000.0

linear_pid = PID(0.05, 0.004, 0.01, setpoint=8)
angular_pid = PID(0.010, 0.002, 0.003, setpoint=256)

def image_callback(msg):
    cv2_img = None
    try:
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)

    hsv_img = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2HSV)
    lower_bound = np.array([20, 100, 100])
    upper_bound = np.array([30, 255, 255])
    yellow_mask = cv2.inRange(hsv_img, lower_bound, upper_bound)

    _, contours, _ = cv2.findContours(yellow_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) == 0:
        return

    max_contour_id = np.argmax([len(c) for c in contours])
    max_contour = contours[max_contour_id]
    if len(max_contour) < CONTOUR_MIN_POINTS:
        return

    M = cv2.moments(max_contour)
    center = np.array((M['m10'] / M['m00'], M['m01'] / M['m00']))
    offsets_from_center = np.linalg.norm(np.array(max_contour).reshape((-1, 2)) - center, axis=1)
    radius = np.median(offsets_from_center)
    dist_from_camera = DIST_CONST / radius

    pub = rospy.Publisher('/vrep/cmd_vel', Twist, queue_size=10)
    twist = Twist()
    twist.linear.x = -linear_pid(dist_from_camera)
    twist.angular.z = -angular_pid(center[0])
    pub.publish(twist)

    # Debug window
    '''
    coords = center.astype(int)
    marked_img = cv2_img.copy()
    cv2.drawContours(marked_img, [max_contour], -1, (0, 0, 255), 2)
    cv2.line(marked_img, tuple(coords + [-6, -6]), tuple(coords + [6, 6]), (0, 0, 255), 2)
    cv2.line(marked_img, tuple(coords + [-6, 6]), tuple(coords + [6, -6]), (0, 0, 255), 2)
    cv2.imshow('debug', marked_img)
    cv2.waitKey(1)

    '''

    # Generate images for report
    '''
    cv2.imwrite('task4_raw.png', cv2_img)
    cv2.imwrite('task4_mask.png', yellow_mask)
    cv2.imwrite('task4_marked.png', marked_img)
    '''

def main():
    rospy.init_node('ball_tracker')
    image_topic = "/vrep/image"
    rospy.Subscriber(image_topic, Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
