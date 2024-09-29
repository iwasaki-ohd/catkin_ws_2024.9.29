#!/usr/bin/env python
# coding: UTF-8

# 2024.1.31
# sortの追跡結果を表示する
# IDを表示
# motpyのコードを移植(世界座標の取得)
# 時刻の追加
# データ統合プログラムに対してパブリッシュ

# bbox.idが空のときはそのデータをパブリッシュしないようにする

from tkinter import Y
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from darknet_ros_msgs.msg import BoundingBoxes
from std_msgs.msg import String, Float32MultiArray
import numpy as np
import cv2
from datetime import datetime

class TrackingImage:
    def __init__(self):
        rospy.init_node("image_subscriber_pc1", anonymous=True)

        # CvBridgeを初期化
        self.bridge = CvBridge()

        # -----------------------追加-----------------------------------
        self.depth_image = None
        self.bounding_boxes_list = []
        self.around = 3 # 距離データの取得範囲
        self.A_cord = np.array([[1.0], [1.0], [1.0], [1.0]]) # 世界座標計算用配列
        # 外部パラメータ行列
        self.extrinsic_parameters = [
                                    [-0.55690952, -0.11869413, -0.82204835,  1.47977405],
                                    [ 0.75217102,  0.34769618, -0.55977327,  2.42841816],
                                    [ 0.35226487, -0.93006401, -0.10435703, -0.13291153],
                                    [ 0.        , 0.         , 0.         , 1.        ]
                                    ]
        """
        外部パラメータのストック
        <cam1>
        旧
        [
            [1.12540220e00, 1.14521609e-01, 2.14551412e-02, 4.41361242e-01],
            [1.02189019e-01, -2.12426572e-01, 9.93322660e-01, -1.74585221e00],
            [-1.52477111e-01, -1.22097908e00, -4.83816232e-01, 1.78920889e00],
            [3.88578059e-16, -3.33066907e-16, 2.22044605e-16, 1.00000000e00],
        ]


        新
        床+壁+ロボ(28点)
        半自動化前はこのパラメータを使用していた(2024.9.23)
        [
            [ 1.07086418e+00, -1.47182142e-02,  4.89235595e-02,  3.54098613e-01],
            [-6.19389769e-02, -4.88021228e-01,  1.02788817e+00, -1.80272089e+00],
            [ 3.00941578e-02, -9.63769763e-01, -4.44642576e-01,  1.92587304e+00],
            [ 2.77555756e-17,  2.49800181e-16, -6.24500451e-17,  1.00000000e+00]
        ]

        <cam2>
        新
        床+ロボ(17点)
        [
            [-2.42738287e-01,  4.05191340e-01, -9.35531480e-01,  2.86083890e+00],
            [ 1.00179605e+00,  6.38568567e-02, -2.50219012e-01,  3.16191196e+00],
            [-2.89507704e-02, -9.43108670e-01, -4.30104576e-01,  2.00400042e+00],
            [ 5.13478149e-16,  2.74086309e-16,  3.33066907e-16,  1.00000000e+00]
        ]

        
        
        """
        self.K_fx = 0
        self.K_fy = 0
        self.K_cx = 0
        self.K_cy = 0
        self.image_w = 0
        self.image_h = 0

        self.multiple_data_pub = rospy.Publisher("multiple_data", String, queue_size=10)# 複数のデータ(ID, 世界座標)を文字列でパブリッシュするパブリッシャ(Unityに送るデータ)
        self.all_data_pub = rospy.Publisher("all_data_pc1", String, queue_size=10) # 統合用データ(時刻, ID, 世界座標)
        self.world_point_pub = rospy.Publisher(
            "world_point", Float32MultiArray, queue_size=10
        )
        self.camera_info_sub = rospy.Subscriber(
            "/camera/aligned_depth_to_color/camera_info",
            CameraInfo,
            self.camera_info_callback,
        )

        self.depth_image_sub = rospy.Subscriber(
            "/camera/aligned_depth_to_color/image_raw", Image, self.depth_image_callback
        )

        # 試しにiouのバウンディングボックスデータを使ってみる(1.7)
        self.bounding_boxes_data_sub = rospy.Subscriber(
            "/pc1/iou_tracker/bounding_boxes",
            BoundingBoxes,
            self.bounding_boxes_data_callback,
        )

        # ------------------------------------------------------------------------------------------------------------

    # デプス画像を表示
    def depth_image_callback(self, data):
        CLmap = None
        self.depth_image = self.bridge.imgmsg_to_cv2(
            data, desired_encoding="passthrough"
        )

        if self.depth_image is not None:
            CLmap = cv2.applyColorMap(
                cv2.convertScaleAbs(self.depth_image, alpha=0.08), cv2.COLORMAP_JET
            )

        for person_box in self.bounding_boxes_list:
            if person_box.Class == "person":
                x_center = int((person_box.xmax + person_box.xmin) / 2)
                y_center = int((person_box.ymax + person_box.ymin) / 2)
                cv2.circle(
                    CLmap,
                    (x_center, y_center),
                    radius=self.around,
                    color=(255, 255, 255),
                    thickness=10,
                )

        cv2.imshow("Depth", CLmap)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            rospy.signal_shutdown("User initiated shutdown")

    # 検出した人物情報を処理する関数
    def bounding_boxes_data_callback(self, data):
        # 時刻を取得
        timestamp = rospy.get_time()
        current_time = datetime.fromtimestamp(timestamp) # 日付と時刻
        #only_time = current_time.time()  # 時刻のみ

        # rospy.loginfo("時刻：{}".format(only_time)) # 確認用

        self.bounding_boxes_list = data.bounding_boxes

        if self.depth_image is None:
            rospy.loginfo("depth_image is None. Skipping bounding_boxes_callback.")
            return

        rospy.loginfo("検出されたオブジェクトの数: {} [個] ".format(len(self.bounding_boxes_list)))
        count = 0
        multiple_data = ""

        for person_box in self.bounding_boxes_list:
            count += 1
            if person_box.Class == "person" and person_box.id is not None: # クラスがperson、かつ、idを持つ場合に処理開始
                x_center = int((person_box.xmax + person_box.xmin) / 2)
                y_center = int((person_box.ymax + person_box.ymin) / 2)

                #if 10 < x_center < 630 and 10 < y_center < 470:
                if 10 < x_center < 1270 and 10 < y_center < 710: # お試し(2024.4.17)

                    rospy.loginfo(
                        "[{}]\n　　　　　　　　　　　　　　オブジェクトID:  {} \n　　　　　　　　　　　　　　bboxの中央座標(x, y): ({}, {})".format(
                            count, person_box.id, x_center, y_center
                        )
                    )

                    depth_values = self.depth_image[
                        y_center - self.around : y_center + self.around,
                        x_center - self.around : x_center + self.around,
                    ]

                    average_depth = np.mean(depth_values) / 1000

                    formatted_average_depth = "{:.5f}".format(average_depth)
                    rospy.loginfo(
                        "カメラとの平均距離: {} [m] ".format(float(formatted_average_depth))
                    )

                    coordinate3D_x = (
                        (x_center - self.image_w / 2) * average_depth / self.K_fx
                    )
                    coordinate3D_y = (
                        (y_center - self.image_h / 2) * average_depth / self.K_fy
                    )

                    rospy.loginfo(
                        "カメラ座標(x, y, z) =  ({}, {}, {})".format(
                            coordinate3D_x, coordinate3D_y, average_depth
                        )
                    )

                    camera_coordinate = [[coordinate3D_x, coordinate3D_y, average_depth]]
                    self.A_cord[0:3, 0] = camera_coordinate[0]
                    world_coordinate = (
                        np.dot(self.extrinsic_parameters, np.array(self.A_cord))[0:3]
                    ).T
                    rospy.loginfo(
                        "世界座標(x, y, z) =  ({}, {}, {})".format(
                            world_coordinate[0][0],
                            world_coordinate[0][1],
                            world_coordinate[0][2],
                        )
                    )

                    world_point = [
                        world_coordinate[0][0],
                        world_coordinate[0][1],
                        world_coordinate[0][2],
                    ]
                    world_point_msg = Float32MultiArray(data=world_point)
                    self.world_point_pub.publish(world_point_msg)

                    formatted_world_coordinate_x = "{:.5f}".format(world_coordinate[0][0])
                    formatted_world_coordinate_y = "{:.5f}".format(world_coordinate[0][1])

                    if count == 1:
                        multiple_data += (
                            str(person_box.id)
                            + (",")
                            + str(formatted_world_coordinate_x)
                            + (",")
                            + str(formatted_world_coordinate_y)
                        )
                    else:
                        multiple_data += (
                            (",")
                            + str(person_box.id)
                            + (",")
                            + str(formatted_world_coordinate_x)
                            + (",")
                            + str(formatted_world_coordinate_y)
                        )

        all_data = f"{current_time}" + (",") + multiple_data  # 日時を含む送信用データ
        if len(multiple_data) > 2:
            self.multiple_data_pub.publish(multiple_data)
            self.all_data_pub.publish(all_data)
            rospy.loginfo("データをパブリッシュ: " + (multiple_data) + "\n ")
            rospy.loginfo("統合用データをパブリッシュ: " + (all_data) + "\n ")  # 確認用

        rospy.loginfo("----------------------------------------")

    # 世界座標の計算に必要な内部パラメータを取得する
    def camera_info_callback(self, msg):
        K = msg.K
        self.K_fx = K[0]
        self.K_fy = K[4]
        self.K_cx = K[2]
        self.K_cy = K[5]
        self.image_w = msg.width
        self.image_h = msg.height

    # -----------------------------------------↑csv_world_coordinate.py-------------------------------------------------------------

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    image_subscriber = TrackingImage()
    try:
        image_subscriber.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node killed!")
        pass
