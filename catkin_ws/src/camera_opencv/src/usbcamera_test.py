#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def main():
    # カメラを起動
    cap = cv2.VideoCapture(0)  # 0番目のカメラを使用
    
    if not cap.isOpened():
        print("カメラが正しく起動できませんでした。")
        return
    
    while True:
        # フレームを取得
        ret, frame = cap.read()
        
        if not ret:
            print("フレームの取得に失敗しました。")
            break
        
        # 画像中央に赤い円を描画
        height, width, _ = frame.shape
        center = (width // 2, height // 2)
        radius = (min(center) - 10) // 5
        color = (0, 0, 255)
        thickness = 2
        cv2.circle(frame, center, radius, color, thickness)
        
        # 変更後の画像を表示
        cv2.imshow('変更後の画像', frame)
        
        # 'q' キーで終了
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    # カメラを解放
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
