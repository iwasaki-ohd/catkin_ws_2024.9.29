#!/usr/bin/env python

# 2024.4.1
# 2024.4.8
# 2024.4.10
# rightShoulderとleftShoulderのキーポイントのみを表示する
# 角度をパブリッシュ7(unity用)
# 切り取り画像の角度を計算

import rospy
from sensor_msgs.msg import Image
from ros_posenet.msg import Poses
from cv_bridge import CvBridge
import cv2
import math
from std_msgs.msg import String
import numpy as np

class PoseVisualizer:
    def __init__(self, line_color=(0, 255, 0), line_thickness=2):
        rospy.init_node('pose_visualizer')
        self.bridge = CvBridge()
        self.image = None  # イメージを初期化
        self.line_color = line_color  # 線の色を指定
        self.line_thickness = line_thickness  # 線の太さを指定
        self.image_sub = rospy.Subscriber('/cut_image_human', Image, self.image_callback)
        self.pose_sub = rospy.Subscriber('/ros_posenet/result', Poses, self.pose_callback)
        self.angle_pub = rospy.Publisher('/shoulder_angle', String, queue_size=10)  # 追加

    def image_callback(self, data):
        image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        # 白い背景の700x600の領域を作成
        self.image = np.ones((600, 700, 3), dtype=np.uint8) * 255
        # 画像を(30, 60)に配置
        #x_offset = (self.image.shape[1] - image.shape[1]) // 2
        #y_offset = (self.image.shape[0] - image.shape[0]) // 2
        x_offset = 30
        y_offset = 60
        self.image[y_offset:y_offset + image.shape[0], x_offset:x_offset + image.shape[1]] = image

    def calculate_shoulder_angle(self, right_shoulder_pos, left_shoulder_pos):
        # 肩の角度計算
        angle_radian = math.atan2(left_shoulder_pos[1] - right_shoulder_pos[1], left_shoulder_pos[0] - right_shoulder_pos[0])
        angle_degree = math.degrees(angle_radian)
        return angle_degree

    def pose_callback(self, data):
        if self.image is None:
            return  # イメージがまだ受信されていない場合は何もしない
        for pose in data.poses:
            right_shoulder = None
            left_shoulder = None
            for keypoint in pose.keypoints:
                part_name = keypoint.part
                # 画像座標
                x = int(keypoint.image_position.x)
                y = int(keypoint.image_position.y)
                # カメラ座標
                cam_x = keypoint.position.x
                cam_z = keypoint.position.z
                if part_name == 'rightShoulder':
                    right_shoulder = (x + 30, y + 60)  # 位置調整
                    cam_right_shoulder = (cam_x, cam_z)
                elif part_name == 'leftShoulder':
                    left_shoulder = (x + 30, y + 60)  # 位置調整
                    cam_left_shoulder = (cam_x, cam_z)
                if right_shoulder is not None and left_shoulder is not None:
                    cv2.circle(self.image, right_shoulder, 5, (0, 255, 0), -1)
                    cv2.circle(self.image, left_shoulder, 5, (0, 255, 0), -1)
                    # テキストを一番上に表示するために、座標を調整
                    text_offset_y = -20  # テキストを上に移動するオフセット
                    cv2.putText(self.image, 'rightShoulder', (right_shoulder[0], right_shoulder[1] + text_offset_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
                    cv2.putText(self.image, 'leftShoulder', (left_shoulder[0], left_shoulder[1] + text_offset_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
                    cv2.line(self.image, right_shoulder, left_shoulder, self.line_color, self.line_thickness)
                    # 角度計算
                    shoulder_angle = int(self.calculate_shoulder_angle(cam_right_shoulder, cam_left_shoulder))
                    if shoulder_angle < 0:
                        shoulder_angle = 360 + shoulder_angle
                    angle_str = 'Shoulder Angle: {:.0f} degrees'.format(shoulder_angle)
                    cv2.putText(self.image, angle_str, (20, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 100, 255), 1, cv2.LINE_AA)
                    self.angle_pub.publish(str(shoulder_angle))  # 人間の方向(角度)をパブリッシュ

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
