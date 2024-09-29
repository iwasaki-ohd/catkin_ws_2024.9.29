#!/usr/bin/env python

# 2023.9.14
# 深度データ確認用

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import pyrealsense2 as rs

depth_scale = None

def depth_image_callback(depth_image):
    global depth_scale

    if depth_scale is None:
        rospy.logerr("Depth scale is not available. Make sure you have correct camera info.")
        return

    # 深度画像データの処理
    try:
        bridge = CvBridge()
        depth_image_cv = bridge.imgmsg_to_cv2(depth_image, desired_encoding="passthrough")
    except CvBridgeError as e:
        print(e)
        return

    # 画像の中央の座標を計算
    height, width = depth_image_cv.shape
    center_x = width // 2
    center_y = height // 2

    # 中央の座標に対応する深度値を取得
    depth_at_center = depth_image_cv[center_y, center_x] * depth_scale  # スケールを適用

    # カメラ座標系での座標を計算
    depth_point = rs.rs2_deproject_pixel_to_point(depth_intrin, [center_x, center_y], depth_at_center)

    # 結果の表示
    print("中央の座標 (ピクセル): ({}, {})".format(center_x, center_y))
    print("中央の深度値 (メートル): {}".format(depth_at_center))
    print("中央の座標 (カメラ座標系): x={}, y={}, z={}".format(depth_point[0], depth_point[1], depth_point[2]))

def camera_info_callback(camera_info):
    global depth_scale
    depth_scale = camera_info.depth_scale

def main():
    global depth_intrin

    rospy.init_node('depth_and_coordinate_publisher', anonymous=True)
    rospy.Subscriber("/camera/depth/image_rect_raw", Image, depth_image_callback)
    rospy.Subscriber("/camera/depth/camera_info", CameraInfo, camera_info_callback)
    
    # カメラ情報から深度スケールを取得
    while depth_scale is None:
        rospy.loginfo("Waiting for camera info...")
        rospy.sleep(1)

    rospy.spin()

if __name__ == '__main__':
    main()
