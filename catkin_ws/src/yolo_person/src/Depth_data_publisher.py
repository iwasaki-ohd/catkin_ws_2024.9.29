#!/usr/bin/env python
# coding: UTF-8

import rospy
from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
import cv2
import numpy as np

depth_image = None
average_depth_pub = None  # パブリッシャのグローバル変数として定義

def depth_image_callback(data):
    global depth_image
    # np.frombufferでデータを格納する
    # reshapeで配列の形状を"data"の高さと幅に変更
    depth_image = np.frombuffer(data.data, dtype=np.uint16).reshape(data.height, data.width)
    
    #確認用
    #rospy.loginfo("データの幅と高さ(w, h): ({}, {})".format(data.height, data.width))
    #rospy.loginfo("\n")

def bounding_boxes_callback(data):
    global depth_image, average_depth_pub
    # "person"のバウンディングボックスを検索
    # "person"のバウンディングボックスデータのみをperspn_boxに格納
    person_box = None
    for box in data.bounding_boxes:
        if box.Class == "person":
            person_box = box
            break

    if person_box is not None:
        # バウンディングボックスの中央座標を計算する
        #x_center = (person_box.xmax + person_box.xmin) // 2
        #y_center = (person_box.ymax + person_box.ymin) // 2
        x_center = int((person_box.xmax + person_box.xmin) / 2 + 165)
        y_center = int((person_box.ymax + person_box.ymin) / 2 + 65)
        # ターミナルに"person"の中央座標を表示する
        rospy.loginfo("人が検出されました。中央座標 (x, y): ({}, {})".format(x_center, y_center))

        # 中央座標の周辺何画素分の取得するかを表す変数
        around = 6
        # バウンディングボックスの中央座標周辺(±around)のピクセル領域の深度情報を取得する
        depth_values = depth_image[y_center-around:y_center+around, x_center-around:x_center+around]
        # 深度情報の平均値を計算する(単位を[m]に調整)
        average_depth = np.mean(depth_values) / 1000
        # ターミナルに深度情報の平均値を表示する
        rospy.loginfo("カメラとの平均距離: {} [m]".format(average_depth))
        rospy.loginfo("\n")

        # 平均深度をパブリッシュする
        average_depth_pub.publish(average_depth)

def main():
    global average_depth_pub  # パブリッシャのグローバル変数を使用するために明示

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

    # パブリッシャの作成
    average_depth_pub = rospy.Publisher("Depth_data", Float32, queue_size=10)

    # ノードを実行し、コールバック関数が呼び出されるまでスピン(ループ)する
    # この関数は、ROSノードがアクティブになり続けるためのメインループを提供する
    rospy.spin()

if __name__ == '__main__':
    main()
