#!/usr/bin/env python
# coding: UTF-8


import rospy
import pyrealsense2 as rs

def main():
    rospy.init_node('realsense_depth')

    # RealSenseのパイプラインを作成
    pipeline = rs.pipeline()
    config = rs.config()

    # 深度ストリームの設定
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    # パイプラインを開始
    profile = pipeline.start(config)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()

    while not rospy.is_shutdown():
        # フレームを待機
        frames = pipeline.wait_for_frames()

        # 深度フレームを取得
        depth_frame = frames.get_depth_frame()

        # 画像の幅と高さを取得
        width = depth_frame.get_width()
        height = depth_frame.get_height()

        # 中央座標を計算
        x = int(width / 2)
        y = int(height / 2)

        # 中央座標の深度データを取得
        depth = depth_frame.get_distance(x, y)

        # ターミナル上に表示
        rospy.loginfo("中央座標の深度: %f [m]", depth)

if __name__ == '__main__':
    try:
        rospy.loginfo("RealSense D435の深度データの取得を開始します")
        main()
    except rospy.ROSInterruptException:
        pass
