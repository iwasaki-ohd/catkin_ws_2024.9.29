#!/usr/bin/env python
# coding: UTF-8

# motpyとセットで実行するプログラム
# 距離データを表示可能な人数は1人

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
    # "face"のバウンディングボックスを検索
    # "face"のバウンディングボックスデータのみをface_boxに格納
    global face_box
    face_box = None
    for box in data.bounding_boxes:
        if box.Class == "face":
            face_box = box
            break

    if face_box is not None:
        # バウンディングボックスの中央座標を計算する
        x_center = (face_box.xmax + face_box.xmin) // 2
        y_center = int((face_box.ymax + face_box.ymin) *480/360 // 2) # サイズ調整(*480/360)
        # ターミナルに"face"の中央座標を表示する
        rospy.loginfo("\n 顔が検出されました。\n オブジェクトID:  {} \n 中央座標(x, y): ({}, {})".format(face_box.id, x_center, y_center))

        # バウンディングボックスの中央座標周辺(±around)のピクセル領域の深度情報を取得する
        #depth_values = depth_image[x_center-around:x_center+around, y_center-around:y_center+around]
        depth_values = depth_image[y_center-around:y_center+around, x_center-around:x_center+around]

        # 深度情報の平均値を計算する(単位を[m]に調整)
        average_depth = np.mean(depth_values) / 1000

        formatted_average_depth = "{:.3f}".format(average_depth)  # 平均深度を小数点第3位までフォーマット
        rospy.loginfo("カメラとの平均距離: {} [m] \n".format(formatted_average_depth))

        # 平均深度をパブリッシュする
        average_depth_pub.publish(average_depth)


def realsensecamera_callback(data):
    global realsemsecamera_cap, face_box
    frame = bridge.imgmsg_to_cv2(data, "bgr8")

    # 顔のバウンディングボックスが検出されていれば、中央座標に円を描画
    if face_box is not None:
        x_center = (face_box.xmax + face_box.xmin) // 2
        y_center = int((face_box.ymax + face_box.ymin) *480/360 // 2) # サイズ調整(*480/360)
        # rospy.loginfo("②顔が検出されました。中央座標 (x, y): ({}, {})".format(x_center, y_center)) # デバック用
        cv2.circle(frame, (x_center, y_center), radius=around, color=(0, 255, 0), thickness=around)

    cv2.imshow('Realsense Camera', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        rospy.signal_shutdown("User initiated shutdown")

def main():
    global average_depth_pub, realsensecamera_cap, face_box

    # ノードの初期化
    rospy.init_node('face_detection_realsense', anonymous=True)

    # サブスクライバの作成
    rospy.Subscriber("/bounding_boxes", BoundingBoxes, bounding_boxes_callback)
    rospy.Subscriber("/camera/depth/image_rect_raw", Image, depth_image_callback)
    rospy.Subscriber("/camera/color/image_raw", Image, realsensecamera_callback)  # Realsenseのカラー映像をサブスクライブ

    # パブリッシャの作成
    average_depth_pub = rospy.Publisher("Depth_data", Float32, queue_size=10)

    # ノードを実行し、コールバック関数が呼び出されるまでスピン(ループ)する
    rospy.spin()

if __name__ == '__main__':
    main()