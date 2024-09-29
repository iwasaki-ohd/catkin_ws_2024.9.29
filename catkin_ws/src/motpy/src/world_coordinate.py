#!/usr/bin/env python
# coding: UTF-8

# 2023.11.5〜
# motpyとセットで実行するプログラム
# 複数人のIDと距離データを表示可能
# 距離データとｘ座標をパブリッシュする
# カメラを中心とした三次元座標の取得
# 外部パラメータを使用した世界座標の計算

from os import error
import rospy
from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
import numpy as np
from cv_bridge import CvBridge
import time
from sensor_msgs.msg import CameraInfo

depth_image = None
multiple_data_pub = None  # 複数のデータ(距離、座標、ID)を文字列で格納する(Unityに送るデータ)
bridge = CvBridge()
bounding_boxes_list = []  # ID表示用
around = 3  #   バウンディングボックスの中央座標の周辺何画素分を取得するかを表す変数(描画する円の半径)

# 外部パラメータ
extrinsic_parameters = [
    [1.12540220e00, 1.14521609e-01, 2.14551412e-02, 4.41361242e-01],
    [1.02189019e-01, -2.12426572e-01, 9.93322660e-01, -1.74585221e00],
    [-1.52477111e-01, -1.22097908e00, -4.83816232e-01, 1.78920889e00],
    [3.88578059e-16, -3.33066907e-16, 2.22044605e-16, 1.00000000e00],
]

"""" 
[1]床のみ
[[ 1.18943255e+00, -3.93790859e-01, -1.53264570e-01,  1.03648060e+00],
[-3.04370208e-02, -3.43451339e+00, -2.60389246e-01,  2.74989249e+00],
[ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00],
[ 2.63677968e-16,  1.77635684e-15,  8.88178420e-16,  1.00000000e+00]]

[5]壁2点(改良版)
[[ 1.12540220e+00,  1.14521609e-01,  2.14551412e-02,  4.41361242e-01],
                         [ 1.02189019e-01, -2.12426572e-01,  9.93322660e-01, -1.74585221e+00],
                         [-1.52477111e-01, -1.22097908e+00, -4.83816232e-01,  1.78920889e+00],
                         [ 3.88578059e-16, -3.33066907e-16,  2.22044605e-16,  1.00000000e+00]]

[3]看板1m                         
[[ 8.90100189e-01,  4.92888330e-01,  1.74336591e-01, -5.70389750e-02],
                         [-1.15407861e-01,  1.39131019e+00,  1.19189661e+00, -2.84963511e+00],
                         [ 4.33014172e-03, -1.65110991e+00, -5.66082477e-01,  2.18598245e+00],
                         [ 4.57966998e-16, -2.22044605e-16,  3.33066907e-16,  1.00000000e+00]]

[2]壁３点                         
[[ 1.13211513e+00,  8.75432316e-02,  2.11163665e-02,  4.43147783e-01],
                        [ 1.41515776e-01, -2.56092463e-01,  9.83655613e-01, -1.72476947e+00],
                        [-1.76954732e-01, -1.18894963e+00, -4.86663092e-01,  1.79640167e+00],
                        [-3.88578059e-16, -9.43689571e-16, -2.77555756e-17,  1.00000000e+00]]
"""
A_cord = np.array([[1.0], [1.0], [1.0], [1.0]])  # 世界座標計算用配列


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
                "[{}]\n                            オブジェクトID:  {} \n                            2Dの中央座標(x, y): ({}, {})".format(
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

            formatted_average_depth = "{:.5f}".format(
                average_depth
            )  # 平均深度を小数点第3位までフォーマット
            rospy.loginfo("カメラとの平均距離: {} [m] ".format(float(formatted_average_depth)))

            # カメラ中心の3D座標に変換
            # coordinate3D_x = ((x_center + image_w/2) - K_cx) * average_depth / K_fx
            # coordinate3D_y = ((y_center + image_h/2) - K_cy) * average_depth / K_fy
            coordinate3D_x = (x_center - image_w / 2) * average_depth / K_fx
            coordinate3D_y = (y_center - image_h / 2) * average_depth / K_fy

            # coordinate3D_x = 0
            # coordinate3D_y = 0

            rospy.loginfo(
                "3D座標(x, y, z) =  ({}, {}, {})".format(
                    coordinate3D_x, coordinate3D_y, average_depth
                )
            )

            # 世界座標の計算
            camera_coordinate = [[coordinate3D_x, coordinate3D_y, average_depth]]
            # camera_coordinate = [[0.026536131277680397, 0.4822809398174286, 2.2039999961853027]] # テスト用

            A_cord[0:3, 0] = camera_coordinate[0]
            world_coordinate = (
                np.dot(extrinsic_parameters, np.array(A_cord))[0:3]
            ).T  # 世界座標の計算 + 転置
            rospy.loginfo(
                "世界座標(x, y, z) =  ({}, {}, {})".format(
                    world_coordinate[0][0],
                    world_coordinate[0][1],
                    world_coordinate[0][2],
                )
            )

            formatted_world_coordinate_x = "{:.5f}".format(
                world_coordinate[0][0]
            )  # 平均深度を小数点第3位までフォーマット
            formatted_world_coordinate_y = "{:.5f}".format(
                world_coordinate[0][1]
            )  # 平均深度を小数点第3位までフォーマット

            if count == 1:
                multiple_data += (
                    str(format(face_box.id, "x"))
                    + (", ")
                    + str(formatted_world_coordinate_x)  # 深度データ
                    + (", ")
                    + str(formatted_world_coordinate_y)  # 世界座標のx
                )
            else:  # ２つ目以降は先頭にカンマを追加
                multiple_data += (
                    (", ")
                    + str(format(face_box.id, "x"))
                    + (", ")
                    + str(formatted_world_coordinate_x)  # 深度データ
                    + (", ")
                    + str(formatted_world_coordinate_y)  # 世界座標のx
                )

    if len(multiple_data) > 2:  # 物体が検出された時にのみパブリッシュ
        multiple_data_pub.publish(multiple_data)
        # time.sleep(0.01)  # ()内の秒数待機

    rospy.loginfo("データをパブリッシュ: " + (multiple_data) + "\n ")  # 確認用(10/5)
    rospy.loginfo("----------------------------------------")


def camera_info_callback(msg):
    global K_fx, K_fy, K_cx, K_cy, image_w, image_h
    #     [fx  0 cx]
    # K = [ 0 fy cy]
    #     [ 0  0  1]
    K = msg.K
    K_fx = K[0]
    K_fy = K[4]
    K_cx = K[2]
    K_cy = K[5]
    image_w = msg.width
    image_h = msg.height

    # rospy.loginfo("取得")
    # rospy.loginfo("D: {}, K: {}, R: {}, P: {}".format(D, K, R, P))
    # rospy.loginfo("K: {}".format(K))
    # rospy.loginfo("K[0]: {}".format(K[0]))
    # rospy.loginfo("width: {}".format(msg.width)) # 640
    # rospy.loginfo("height: {}".format(msg.height)) # 480


def main():
    global multiple_data_pub

    # ノードの初期化
    rospy.init_node("realsense-ros-unity", anonymous=True)

    # パブリッシャの作成（Unity送信用)
    multiple_data_pub = rospy.Publisher("multiple_data", String, queue_size=10)

    # サブスクライバの作成
    rospy.Subscriber(
        "/camera/aligned_depth_to_color/camera_info", CameraInfo, camera_info_callback
    )
    rospy.Subscriber(
        "/camera/aligned_depth_to_color/image_raw", Image, depth_image_callback
    )
    rospy.Subscriber("/bounding_boxes", BoundingBoxes, bounding_boxes_callback)

    # ノードを実行し、コールバック関数が呼び出されるまでスピン(ループ)する
    rospy.spin()


if __name__ == "__main__":
    main()
