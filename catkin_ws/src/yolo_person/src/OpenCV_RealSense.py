#!/usr/bin/env python
# coding: UTF-8

# yolo.v3とセットで実行するプログラム

import rospy
from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
import cv2
import numpy as np
from cv_bridge import CvBridge

depth_image = None
average_depth_pub = None  # パブリッシャのグローバル変数として定義
bridge = CvBridge()
realsensecamera_cap = None  # Realsenseカメラのキャプチャーオブジェクト

around = 3 #   バウンディングボックスの中央座標の周辺何画素分を取得するかを表す変数(描画する円の半径)

def depth_image_callback(data):
    global depth_image
    # np.frombufferでデータを格納する
    # reshapeで配列の形状を"data"の高さと幅に変更
    # 画面の大きさは848*480
    depth_image = np.frombuffer(data.data, dtype=np.uint16).reshape(data.height, data.width)

def bounding_boxes_callback(data):
    global depth_image, average_depth_pub
    # "person"のバウンディングボックスを検索
    # "person"のバウンディングボックスデータのみをperspn_boxに格納
    global person_box
    person_box = None
    for box in data.bounding_boxes:
        if box.Class == "person":
            person_box = box
            break

    if person_box is not None:
        # バウンディングボックスの中央座標を計算する
        #x_center = int((person_box.xmax + person_box.xmin) * 848/640 / 2)
        x_center = int((person_box.xmax + person_box.xmin) / 2)
        y_center = (person_box.ymax + person_box.ymin) // 2
        # ターミナルに"person"の中央座標を表示する
        rospy.loginfo("人が検出されました。中央座標 (x, y): ({}, {})".format(x_center, y_center))

        # バウンディングボックスの中央座標周辺(±around)のピクセル領域の深度情報を取得する
        depth_values = depth_image[x_center-around:x_center+around, y_center-around:y_center+around]
        # 深度情報の平均値を計算する(単位を[m]に調整)
        average_depth = np.mean(depth_values) / 1000
        # ターミナルに深度情報の平均値を表示する
        rospy.loginfo("カメラとの平均距離: {} [m]".format(average_depth))
        rospy.loginfo("\n")

        # 平均深度をパブリッシュする
        average_depth_pub.publish(average_depth)


def realsensecamera_callback(data):
    global realsemsecamera_cap, person_box
    frame = bridge.imgmsg_to_cv2(data, "bgr8")

    # 人物のバウンディングボックスが検出されていれば、中央座標に円を描画
    if person_box is not None:
        x_center = (person_box.xmax + person_box.xmin) // 2
        y_center = (person_box.ymax + person_box.ymin) // 2
        cv2.circle(frame, (x_center, y_center), radius=around, color=(0, 255, 0), thickness=3)

    cv2.imshow('Realsense Camera', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        rospy.signal_shutdown("User initiated shutdown")

def main():
    global average_depth_pub, realsensecamera_cap, person_box

    # ノードの初期化
    rospy.init_node('person_detection_realsense', anonymous=True)

    # サブスクライバの作成
    rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, bounding_boxes_callback)
    rospy.Subscriber("/camera/depth/image_rect_raw", Image, depth_image_callback)
    rospy.Subscriber("/camera/color/image_raw", Image, realsensecamera_callback)  # Realsenseのカラー映像をサブスクライブ

    # パブリッシャの作成
    average_depth_pub = rospy.Publisher("Depth_data", Float32, queue_size=10)

    # ノードを実行し、コールバック関数が呼び出されるまでスピン(ループ)する
    rospy.spin()

if __name__ == '__main__':
    main()
