#!/usr/bin/env python
# coding: UTF-8

import rospy
from sensor_msgs.msg import Image
from ros_posenet.msg import Keypoint
from cv_bridge import CvBridge
import cv2

class PoseVisualizer:
    def __init__(self):
        self.cv_bridge = CvBridge()

        # ROS topics
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        self.keypoints_sub = rospy.Subscriber("/ros_posenet/result", Keypoint, self.keypoints_callback)

        # OpenCV window
        cv2.namedWindow("Pose Visualization")

        # Initialize variables
        self.image = None
        self.keypoints = []

    def image_callback(self, data):
        self.image = self.cv_bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")

    def keypoints_callback(self, data):
        # Append the new keypoints
        self.keypoints.append(data)

    def visualize(self):
        while not rospy.is_shutdown():
            if self.image is not None:
                image_to_display = self.image.copy()
                # Display the image with keypoints
                cv2.imshow("Pose Visualization", image_to_display)
                cv2.waitKey(1)

                # Draw keypoints on the image and display part name
                for keypoint in self.keypoints:
                    x = int(keypoint.image_position.x)
                    y = int(keypoint.image_position.y)
                    part_name = keypoint.part
                    cv2.circle(image_to_display, (x, y), 5, (0, 255, 0), -1)
                    cv2.putText(image_to_display, part_name, (x + 10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                
                # Print all keypoints information
                rospy.loginfo("Received keypoints:")
                for keypoint in self.keypoints:
                    rospy.loginfo("Part {}: x={}, y={}, score={}".format(keypoint.part, keypoint.image_position.x, keypoint.image_position.y, keypoint.score))
                self.keypoints = []  # Clear keypoints after printing them

if __name__ == '__main__':
    rospy.init_node('pose_visualizer')
    pose_visualizer = PoseVisualizer()
    pose_visualizer.visualize()
