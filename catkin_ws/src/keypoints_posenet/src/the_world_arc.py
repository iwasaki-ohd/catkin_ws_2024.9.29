#!/usr/bin/env python

# 2024.4.1
# 2024.4.8
# rightShoulderとleftShoulderのキーポイントのみを表示する
# 角度をパブリッシュ7(unity用)

# 2024.4.22
# 世界座標に基づく角度

import rospy
from sensor_msgs.msg import Image
from ros_posenet.msg import Poses
from cv_bridge import CvBridge
import cv2
import math
from std_msgs.msg import String
import numpy as np
import copy

class PoseVisualizer:
    def __init__(self, line_color=(0, 255, 0), line_thickness=2):
        rospy.init_node('pose_visualizer')
        self.bridge = CvBridge()
        self.image = None  # イメージを初期化
        self.line_color = line_color  # 線の色を指定
        self.line_thickness = line_thickness  # 線の太さを指定
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.pose_sub = rospy.Subscriber('/ros_posenet/result', Poses, self.pose_callback)
        self.angle_pub = rospy.Publisher('/shoulder_angle', String, queue_size=10)  # 追加

        # 世界座標計算用
        self.A_cord = np.array([[1.0], [1.0], [1.0], [1.0]]) # 世界座標計算用配列
        # 外部パラメータ行列
        self.extrinsic_parameters = [
            [ 1.07086418e+00, -1.47182142e-02,  4.89235595e-02,  3.54098613e-01],
            [-6.19389769e-02, -4.88021228e-01,  1.02788817e+00, -1.80272089e+00],
            [ 3.00941578e-02, -9.63769763e-01, -4.44642576e-01,  1.92587304e+00],
            [ 2.77555756e-17,  2.49800181e-16, -6.24500451e-17,  1.00000000e+00]
        ]

    def image_callback(self, data):
        self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")

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
                cam_y = keypoint.position.y
                cam_z = keypoint.position.z
                if part_name == 'rightShoulder':
                    right_shoulder = (x, y)
                    camera_coordinate = [[cam_x, cam_y, cam_z]]
                    X_cord = copy.deepcopy(self.A_cord)
                    X_cord[0:3, 0] = camera_coordinate[0]
                    world_coordinate = (np.dot(self.extrinsic_parameters, np.array(X_cord))[0:3]).T
                    rospy.loginfo("rightの世界座標(x, y, z) =  ({}, {}, {})".format(world_coordinate[0][0],world_coordinate[0][1],world_coordinate[0][2]))
                    world_point = [
                    world_coordinate[0][0],
                    world_coordinate[0][1],
                    world_coordinate[0][2],
                    ]
                    world_right_shoulder = (world_point[0], world_point[2])
                elif part_name == 'leftShoulder':
                    left_shoulder = (x, y)
                    camera_coordinate = [[cam_x, cam_y, cam_z]]
                    X_cord = copy.deepcopy(self.A_cord)
                    X_cord[0:3, 0] = camera_coordinate[0]
                    world_coordinate = (np.dot(self.extrinsic_parameters, np.array(X_cord))[0:3]).T
                    rospy.loginfo("leftの世界座標(x, y, z) =  ({}, {}, {})".format(world_coordinate[0][0],world_coordinate[0][1],world_coordinate[0][2]))
                    world_point = [
                    world_coordinate[0][0],
                    world_coordinate[0][1],
                    world_coordinate[0][2],
                    ]
                    world_left_shoulder = (world_point[0], world_point[2])
                if right_shoulder is not None and left_shoulder is not None:
                    #rospy.loginfo("通った！！！！！！")
                    cv2.circle(self.image, right_shoulder, 5, (0, 255, 0), -1)
                    cv2.circle(self.image, left_shoulder, 5, (0, 255, 0), -1)
                    # テキストを一番上に表示するために、座標を調整
                    text_offset_y = -20  # テキストを上に移動するオフセット
                    cv2.putText(self.image, 'rightShoulder', (right_shoulder[0], right_shoulder[1] + text_offset_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
                    cv2.putText(self.image, 'leftShoulder', (left_shoulder[0], left_shoulder[1] + text_offset_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
                    cv2.line(self.image, right_shoulder, left_shoulder, self.line_color, self.line_thickness)
                    # 角度計算
                    #rospy.loginfo(f"right:{world_right_shoulder}, left:{world_left_shoulder}")
                    shoulder_angle = int(self.calculate_shoulder_angle(world_left_shoulder, world_right_shoulder))
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
