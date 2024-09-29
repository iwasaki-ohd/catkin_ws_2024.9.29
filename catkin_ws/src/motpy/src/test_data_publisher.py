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
import time
import random


def main():
    global Coordinate_X_pub, Coordinate_Y_pub, realsensecamera_cap, face_box

    # ノードの初期化
    rospy.init_node("Data_pub", anonymous=True)

    # パブリッシャの作成（Unity送信用)
    Coordinate_X_pub = rospy.Publisher("Coordinate_X", Float32, queue_size=10)
    Coordinate_Y_pub = rospy.Publisher("Coordinate_Y", Float32, queue_size=10)

    while not rospy.is_shutdown():
        Coordinate = [round(random.uniform(0.5, 1.3), 2), random.randint(600, 640)]
        rospy.loginfo(
            "データをパブリッシュ: ({}, {}) ".format(Coordinate[0], Coordinate[1])
        )  # 確認用(9/28)

        # パブリッシュする
        Coordinate_X_pub.publish(Coordinate[0])
        Coordinate_Y_pub.publish(Coordinate[1])
        time.sleep(3)  # ()内の秒数待機


if __name__ == "__main__":
    main()
