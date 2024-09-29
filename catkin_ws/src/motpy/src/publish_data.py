#!/usr/bin/env python
# coding: UTF-8

# 2023.9.27〜
# motpyとセットで実行するプログラム
# 複数人のIDと距離データを表示可能

# 距離データとｘ座標をパブリッシュする

import rospy
from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
import cv2
import numpy as np
from cv_bridge import CvBridge

depth_image = None
Coordinate_X_pub = None  # パブリッシャのグローバル変数として定義(Unityに送るデータ)
Coordinate_Y_pub = None  # パブリッシャのグローバル変数として定義(Unityに送るデータ)
bridge = CvBridge()
realsensecamera_cap = None  # Realsenseカメラのキャプチャーオブジェクト
bounding_boxes_list = [] # ID表示用

around = 3 #   バウンディングボックスの中央座標の周辺何画素分を取得するかを表す変数(描画する円の半径)

# "/camera/depth/image_rect_raw"のデータを使用
def depth_image_callback(data):
    global depth_image
    # np.frombufferでデータを格納する
    # reshapeで配列の形状を"data"の高さと幅に変更
    # 画面の大きさは848*480
    # depth_image = np.frombuffer(data.data, dtype=np.uint16).reshape(data.height, data.width)
    depth_image = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")

# "/bounding_boxes"からデータを取得して使用する
def bounding_boxes_callback(data):
    global depth_image, Coordinate_X_pub, Coordinate_Y_pub, bounding_boxes_list

    # バウンディングボックス情報をリストに格納する
    bounding_boxes_list = data.bounding_boxes

    rospy.loginfo("検出されたオブジェクトの数: {} [個] ".format(len(bounding_boxes_list)))
    # リスト内の各バウンディングボックス情報に対して処理を行う
    count = 0 # オブジェクト番号(認識順)
    for face_box in bounding_boxes_list:
        count += 1
        if face_box.Class == "face":
            # バウンディングボックスの中央座標を計算する
            #x_center = int((face_box.xmax + face_box.xmin) / 2)
            #y_center = int((face_box.ymax + face_box.ymin) / 2) # 確認用
            x_center = int((face_box.xmax + face_box.xmin) / 2 * 440 / 640 + 200) # サイズ調整
            y_center = int((face_box.ymax + face_box.ymin) / 2 *340/480  + 60) # サイズ調整

            # ターミナルに各オブジェクトのIDと中央座標を表示する
            rospy.loginfo("[{}]\n                            オブジェクトID:  {} \n                            中央座標(x, y): ({}, {})".format(count, format(face_box.id, 'x'), x_center, y_center))

            # バウンディングボックスの中央座標周辺(±around)のピクセル領域の深度情報を取得する
            depth_values = depth_image[y_center-around:y_center+around, x_center-around:x_center+around]

            # 深度情報の平均値を計算する(単位を[m]に調整)
            average_depth = np.mean(depth_values) / 1000

            formatted_average_depth = "{:.3f}".format(average_depth)  # 平均深度を小数点第3位までフォーマット
            rospy.loginfo("カメラとの平均距離: {} [m] ".format(float(formatted_average_depth)))

            Coordinate = [float(formatted_average_depth), x_center]
            rospy.loginfo("データをパブリッシュ: ({}, {}) ".format(Coordinate[0], Coordinate[1])) # 確認用(9/28)


            # パブリッシュする
            Coordinate_X_pub.publish(Coordinate[0])
            Coordinate_Y_pub.publish(Coordinate[1])
        
    rospy.loginfo("----------------------------------------")



# "/camera/color/image_raw"のデータを使用
def realsensecamera_callback(data):
    global realsemsecamera_cap, face_box, depth_image
    frame = bridge.imgmsg_to_cv2(data, "bgr8") # カラー画像を取得
    #depth_image = bridge.imgmsg_to_cv2(data, "bgr8")

    # 顔のバウンディングボックスが検出されていれば、中央座標に円を描画
    # リスト内の各バウンディングボックス情報に対して処理を行う
    for face_box in bounding_boxes_list:
        if face_box.Class == "face":
            x_center = int((face_box.xmax + face_box.xmin) / 2 *440/640 + 200) # サイズ調整
            y_center = int((face_box.ymax + face_box.ymin) / 2 *340 / 480 + 60) # サイズ調整
            x_rs = int((face_box.xmax + face_box.xmin) / 2)
            # rospy.loginfo("②顔が検出されました。中央座標 (x, y): ({}, {})".format(x_center, y_center)) # デバック用
            cv2.circle(depth_image, (x_center, y_center), radius=around, color=(20, 20, 20), thickness=10)
            # cv2.circle(frame, (x_rs, y_center), radius=around, color=(0, 255, 0), thickness=10)
        
    if depth_image is not None:
        # Depthデータを10-200の範囲に正規化
        depth_image = cv2.normalize(depth_image, None, 30, 255, cv2.NORM_MINMAX)
        # 距離の近いものを黒く、遠いものを白くする(値を絶対値に変える)
        depth_image = cv2.convertScaleAbs(depth_image)

    cv2.imshow('G', depth_image) # 距離に基づいたグレースケール表示
    # cv2.imshow('Realsense Camera', frame)  # カラー画像

    if cv2.waitKey(1) & 0xFF == ord('q'):
        rospy.signal_shutdown("User initiated shutdown")

def main():
    global Coordinate_X_pub, Coordinate_Y_pub, realsensecamera_cap, face_box

    # ノードの初期化
    rospy.init_node('realsense-ros-unity', anonymous=True)

    # サブスクライバの作成
    rospy.Subscriber("/bounding_boxes", BoundingBoxes, bounding_boxes_callback)
    rospy.Subscriber("/camera/depth/image_rect_raw", Image, depth_image_callback)
    rospy.Subscriber("/camera/color/image_raw", Image, realsensecamera_callback)  # Realsenseのカラー映像をサブスクライブ

    # パブリッシャの作成（Unity送信用)
    Coordinate_X_pub = rospy.Publisher("Coordinate_X", Float32, queue_size=10)
    Coordinate_Y_pub = rospy.Publisher("Coordinate_Y", Float32, queue_size=10)

    # ノードを実行し、コールバック関数が呼び出されるまでスピン(ループ)する
    rospy.spin()

if __name__ == '__main__':
    main()
