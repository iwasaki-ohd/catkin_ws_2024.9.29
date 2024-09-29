#!/usr/bin/env python

import rospy
import cv2
import numpy as np

if __name__ == '__main__':
    # ROSノードの初期化
    rospy.init_node('opencv_example_node')
    
    # 画像の読み込み
    image = cv2.imread('/home/iwasaki/ピクチャ/image.png')
    if image is None:
        rospy.logerr('画像の読み込みに失敗しました')
    else:
        # 画像を600x400にリサイズ
        resized_image = cv2.resize(image, (600, 400))
        
        # 画像中央に赤い円を描画
        height, width, _ = resized_image.shape
        center = (width // 2, height // 2)
        radius = (min(center) - 10) // 5  # 円の半径（画像の短辺から少し引いた値を定数で割った値）
        color = (200, 100, 0)  # 赤色（BGR形式）
        thickness = 2
        cv2.circle(resized_image, center, radius, color, thickness)
        
        # 変更後の画像を表示
        cv2.namedWindow('変更後の画像', cv2.WINDOW_NORMAL) # ウィンドウのリサイズを可能にする
        cv2.imshow('変更後の画像', resized_image)
        cv2.waitKey(0) # キーを打ったら画像の表示を終了する
