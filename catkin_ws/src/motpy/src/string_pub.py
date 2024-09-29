#!/usr/bin/env python
# coding: UTF-8

# 2023.9.27〜
# motpyとセットで実行するプログラム
# 複数人のIDと距離データを表示可能

# 距離データとｘ座標をパブリッシュする

from ctypes import resize
from os import error
import rospy
from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from std_msgs.msg import String
import cv2
import numpy as np
from cv_bridge import CvBridge
import time

depth_image = None
multiple_data_pub = None  # 複数のデータ(距離、座標、ID)を文字列で格納する(Unityに送るデータ)
bridge = CvBridge()
bounding_boxes_list = []  # ID表示用
around = 3  #   バウンディングボックスの中央座標の周辺何画素分を取得するかを表す変数(描画する円の半径)


def depth_image_callback(data):
    global face_box, depth_image
    CLmap = None

    depth_image = bridge.imgmsg_to_cv2(
        data, desired_encoding="passthrough"
    )  # トピックからデータを取得

    if depth_image is not None:
        # depth imageをカラーマップに変換
        CLmap = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_image, alpha=0.08), cv2.COLORMAP_JET
        )

    # 顔のバウンディングボックスが検出されていれば、中央座標に円を描画
    # リスト内の各バウンディングボックス情報に対して処理を行う
    for face_box in bounding_boxes_list:
        if face_box.Class == "face":
            x_center = int((face_box.xmax + face_box.xmin) / 2)
            y_center = int((face_box.ymax + face_box.ymin) / 2)
            cv2.circle(
                CLmap,
                (x_center, y_center),
                radius=around,
                color=(255, 255, 255),
                thickness=10,
            )  # 深度取得座標に円を描画

    cv2.imshow("Depth", CLmap)  # 距離に基づいたカラー画像を表示

    if cv2.waitKey(1) & 0xFF == ord("q"):
        rospy.signal_shutdown("User initiated shutdown")


# "/bounding_boxes"からデータを取得して使用する
def bounding_boxes_callback(data):
    global depth_image, bounding_boxes_list, multiple_data_pub

    # バウンディングボックス情報をリストに格納する
    bounding_boxes_list = data.bounding_boxes

    # depth_imageがNoneの場合、エラーメッセージを表示して関数を終了
    if depth_image is None:
        # rospy.logerr("depth_image is None. Skipping bounding_boxes_callback.")
        rospy.loginfo("depth_image is None. Skipping bounding_boxes_callback.")
        return

    rospy.loginfo("検出されたオブジェクトの数: {} [個] ".format(len(bounding_boxes_list)))
    # リスト内の各バウンディングボックス情報に対して処理を行う
    count = 0  # オブジェクト番号(認識順)
    multiple_data = ""  # 各データを格納する文字列

    for face_box in bounding_boxes_list:
        count += 1
        if face_box.Class == "face":
            # バウンディングボックスの中央座標を計算する
            x_center = int((face_box.xmax + face_box.xmin) / 2)
            y_center = int((face_box.ymax + face_box.ymin) / 2)

            # ターミナルに各オブジェクトのIDと中央座標を表示する
            rospy.loginfo(
                "[{}]\n                            オブジェクトID:  {} \n                            中央座標(x, y): ({}, {})".format(
                    count, format(face_box.id, "x"), x_center, y_center
                )
            )

            # バウンディングボックスの中央座標周辺(±around)のピクセル領域の深度情報を取得する
            depth_values = depth_image[
                y_center - around : y_center + around,
                x_center - around : x_center + around,
            ]

            # 深度情報の平均値を計算する(単位を[m]に調整)
            average_depth = np.mean(depth_values) / 1000

            formatted_average_depth = "{:.3f}".format(
                average_depth
            )  # 平均深度を小数点第3位までフォーマット
            rospy.loginfo("カメラとの平均距離: {} [m] ".format(float(formatted_average_depth)))

            if count == 1:
                multiple_data += (
                    str(count)
                    + (", ")
                    + str(formatted_average_depth)
                    + (", ")
                    + str(x_center)
                )
            else:  # ２つ目以降は先頭にカンマを追加
                multiple_data += (
                    (", ")
                    + str(count)
                    + (", ")
                    + str(formatted_average_depth)
                    + (", ")
                    + str(x_center)
                )

    if len(multiple_data) > 2:  # 物体が検出された時にのみパブリッシュ
        multiple_data_pub.publish(multiple_data)
        # time.sleep(0.01)  # ()内の秒数待機

    rospy.loginfo("Stringデータをパブリッシュ: " + (multiple_data) + "\n ")  # 確認用(10/5)
    rospy.loginfo("----------------------------------------")


def main():
    global multiple_data_pub

    # ノードの初期化
    rospy.init_node("realsense-ros-unity", anonymous=True)

    # パブリッシャの作成（Unity送信用)
    multiple_data_pub = rospy.Publisher("multiple_data", String, queue_size=10)

    # サブスクライバの作成
    rospy.Subscriber(
        "/camera/aligned_depth_to_color/image_raw", Image, depth_image_callback
    )
    rospy.Subscriber("/bounding_boxes", BoundingBoxes, bounding_boxes_callback)

    # ノードを実行し、コールバック関数が呼び出されるまでスピン(ループ)する
    rospy.spin()


if __name__ == "__main__":
    main()
