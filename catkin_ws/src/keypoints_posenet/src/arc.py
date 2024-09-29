#!/usr/bin/env python

# 2024.4.1
# rightShoulderとleftShoulderのキーポイントのみを表示する

import rospy
from sensor_msgs.msg import Image
from ros_posenet.msg import Poses 
from cv_bridge import CvBridge
import cv2

class PoseVisualizer:
    def __init__(self, line_color=(0, 255, 0), line_thickness=2):
        rospy.init_node('pose_visualizer')
        self.bridge = CvBridge()
        self.image = None  # イメージを初期化
        self.line_color = line_color  # 線の色を指定
        self.line_thickness = line_thickness  # 線の太さを指定
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.pose_sub = rospy.Subscriber('/ros_posenet/result', Poses, self.pose_callback)

    def image_callback(self, data):
        self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")

    def pose_callback(self, data):
        if self.image is None:
            return  # イメージがまだ受信されていない場合は何もしない
        for pose in data.poses:
            right_shoulder = None
            left_shoulder = None
            for keypoint in pose.keypoints:
                part_name = keypoint.part
                x = int(keypoint.image_position.x)
                y = int(keypoint.image_position.y)
                if part_name == 'rightShoulder':
                    right_shoulder = (x, y)
                elif part_name == 'leftShoulder':
                    left_shoulder = (x, y)
                if right_shoulder is not None and left_shoulder is not None:
                    cv2.circle(self.image, right_shoulder, 5, (0, 255, 0), -1)
                    cv2.circle(self.image, left_shoulder, 5, (0, 255, 0), -1)
                    # テキストを一番上に表示するために、座標を調整
                    text_offset_y = -20  # テキストを上に移動するオフセット
                    cv2.putText(self.image, 'rightShoulder', (right_shoulder[0], right_shoulder[1] + text_offset_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
                    cv2.putText(self.image, 'leftShoulder', (left_shoulder[0], left_shoulder[1] + text_offset_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
                    cv2.line(self.image, right_shoulder, left_shoulder, self.line_color, self.line_thickness)

        cv2.imshow("Pose Visualization", self.image)
        cv2.waitKey(1)

if __name__ == '__main__':
    try:
        # ユーザーが線の色と太さを指定できるようにする
        line_color = (255, 0, 0)  # 例: 青色
        line_thickness = 3  # 例: 太さ3
        visualizer = PoseVisualizer(line_color=line_color, line_thickness=line_thickness)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

