#!/usr/bin/env python
# coding: UTF-8

# 2024.3.27
# posenetの結果を可視化するテストプログラム1号
# 表示可能部位は一箇所


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
        self.keypoints = {}
        self.keypoints_last_received = {}

    def image_callback(self, data):
        self.image = self.cv_bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")

    def keypoints_callback(self, data):
        # Store keypoints based on part name
        part_name = data.part
        self.keypoints[part_name] = data
        self.keypoints_last_received[part_name] = rospy.get_time()

    def visualize(self):
        while not rospy.is_shutdown():
            if self.image is not None:
                image_to_display = self.image.copy()

                # Remove keypoints not received within last 15 messages
                current_time = rospy.get_time()
                for part_name, last_received_time in self.keypoints_last_received.items():
                    if current_time - last_received_time > 15:
                        if part_name in self.keypoints:
                            del self.keypoints[part_name]

                # Draw keypoints on the image and display part name
                for part_name in self.keypoints:
                    keypoint = self.keypoints[part_name]
                    x = int(keypoint.image_position.x)
                    y = int(keypoint.image_position.y)
                    cv2.circle(image_to_display, (x, y), 5, (0, 255, 0), -1)
                    cv2.putText(image_to_display, part_name, (x + 10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

                # Display the image with keypoints
                cv2.imshow("Pose Visualization", image_to_display)
                cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('pose_visualizer')
    pose_visualizer = PoseVisualizer()
    pose_visualizer.visualize()
