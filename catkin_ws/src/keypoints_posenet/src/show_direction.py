#!/usr/bin/env python

# 2024.3.28
# 全てのキーポイントを表示する

import rospy
from sensor_msgs.msg import Image
from ros_posenet.msg import Poses 
from cv_bridge import CvBridge
import cv2

class PoseVisualizer:
    def __init__(self):
        rospy.init_node('pose_visualizer')
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.pose_sub = rospy.Subscriber('/ros_posenet/result', Poses, self.pose_callback)

    def image_callback(self, data):
        self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")

    def pose_callback(self, data):
        eyes_detected = {'left': False, 'right': False}
        for pose in data.poses:
            for keypoint in pose.keypoints:
                part_name = keypoint.part
                x = int(keypoint.image_position.x)
                y = int(keypoint.image_position.y)
                cv2.circle(self.image, (x, y), 5, (0, 255, 0), -1)
                cv2.putText(self.image, part_name, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
                
                if part_name == 'leftEye':
                    eyes_detected['left'] = True
                elif part_name == 'rightEye':
                    eyes_detected['right'] = True

        if eyes_detected['left'] and eyes_detected['right']:
            cv2.putText(self.image, "Front", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
        elif eyes_detected['left']:
            cv2.putText(self.image, "Left", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
        elif eyes_detected['right']:
            cv2.putText(self.image, "Right", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
        else:
            cv2.putText(self.image, "Back", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)

        cv2.imshow("Pose Visualization", self.image)
        cv2.waitKey(1)

if __name__ == '__main__':
    try:
        visualizer = PoseVisualizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

