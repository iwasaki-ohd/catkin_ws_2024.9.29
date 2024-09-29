#!/usr/bin/env python
# coding: UTF-8

# 距離データ取得位置の修正(デプス画像調整)
# クリック点の取得
# 2024.1.7

import pyrealsense2 as rs
import numpy as np
import cv2


def get_click_point(event, x, y, flags, param):
    global depth_frame

    if event == cv2.EVENT_LBUTTONDOWN:
        if x >= 640:
            return

        depth = depth_frame.get_distance(x, y)
        point = np.array([x, y, depth])
        if depth == 0:
            print("ERROR: cannot get depth data ! ")
            return

        x = point[0]
        y = point[1]
        z = point[2]

        x, y, z = rs.rs2_deproject_pixel_to_point(color_intrinsics, [x, y], z)

        print("point:", [x, y, z])


# ストリーム(Depth/Color)の設定
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
#
#config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
#config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
#
#config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
#config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

# ストリーミング開始
pipeline = rs.pipeline()
profile = pipeline.start(config)

# Alignオブジェクト生成
align_to = rs.stream.color
align = rs.align(align_to)


try:
    while True:
        # フレーム待ち(Color & Depth)
        frames = pipeline.wait_for_frames()

        aligned_frames = align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        color_intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
        if not depth_frame or not color_frame:
            continue

        # imageをnumpy arrayに
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        # depth imageをカラーマップに変換
        depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_image, alpha=0.022), cv2.COLORMAP_JET
        )

        # 画像表示
        color_image_s = cv2.resize(color_image, (640, 480))
        depth_colormap_s = cv2.resize(depth_colormap, (640, 480))
        #color_image_s = cv2.resize(color_image, (1280, 720))
        #depth_colormap_s = cv2.resize(depth_colormap, (1280, 720))
        images = np.hstack((color_image_s, depth_colormap_s))
        cv2.namedWindow("RealSense", cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback("RealSense", get_click_point)
        cv2.imshow("RealSense", images)

        if cv2.waitKey(1) & 0xFF == 27:  # ESCで終了
            cv2.destroyAllWindows()
            break

finally:
    # ストリーミング停止
    pipeline.stop()
