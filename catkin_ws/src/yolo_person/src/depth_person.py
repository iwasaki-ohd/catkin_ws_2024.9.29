#!/usr/bin/env python
# coding: UTF-8

import rospy
from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import Image
import cv2
import numpy as np

depth_image = None

def depth_image_callback(data):
    global depth_image
    # np.frombufferでデータを格納する
    # reshapeで配列の形状を"data"の高さと幅に変更
    depth_image = np.frombuffer(data.data, dtype=np.uint16).reshape(data.height, data.width)

def bounding_boxes_callback(data):
    global depth_image
    # "person"のバウンディングボックスを検索
    # "person"のバウンディングボックスデータのみをperspn_boxに格納
    person_box = None
    for box in data.bounding_boxes:
        if box.Class == "person":
            person_box = box
            break

    if person_box is not None:
        # バウンディングボックスの中央座標を計算する
        x_center = (person_box.xmax + person_box.xmin) // 2
        y_center = (person_box.ymax + person_box.ymin) // 2
        # ターミナルに"person"の中央座標を表示する
        rospy.loginfo("人が検出されました。中央座標 (x, y): ({}, {})".format(x_center, y_center))

        # 中央座標の周辺何画素分の取得するかを表す変数
        around = 10
        # バウンディングボックスの中央座標周辺(±around)のピクセル領域の深度情報を取得する
        depth_values = depth_image[x_center-around:x_center+around, y_center-around:y_center+around]
        # 深度情報の平均値を計算する(単位を[m]に調整)
        average_depth = np.mean(depth_values) / 1000
        # ターミナルに深度情報の平均値を表示する
        rospy.loginfo("カメラとの平均距離: {} [m]".format(average_depth))

def main():
    # ノードの初期化
    rospy.init_node('person_detection', anonymous=True)

    # サブスクライバの作成
    # BoundingBoxesメッセージのサブスクライブ
    # /darknet_ros/bounding_boxesというトピックに対してのサブスクライバ
    # 物体検出結果のメッセージが受信されると、bounding_boxes_callback関数が呼び出される
    rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, bounding_boxes_callback)

    # Depth画像のサブスクライブ
    # /camera/depth/image_rect_rawというトピックに対してのサブスクライバ
    # 深度画像のメッセージが受信されると、depth_image_callback関数が呼び出される
    rospy.Subscriber("/camera/depth/image_rect_raw", Image, depth_image_callback)

    # ノードを実行し、コールバック関数が呼び出されるまでスピン(ループ)する
    # この関数は、ROSノードがアクティブになり続けるためのメインループを提供する
    rospy.spin()

if __name__ == '__main__':
    main()
