#!/usr/bin/env python
# coding: UTF-8

import cv2
import numpy as np
import rospy
from std_msgs.msg import String
from datetime import datetime
import math

# グローバル変数
drawing = False  # 描画中かどうか
ix, iy = -1, -1  # 始点座標
path = []  # 描画された座標リスト

# マウスコールバック関数
def draw_path(event, x, y, flags, param):
    global ix, iy, drawing, path

    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        ix, iy = x, y
        path = [(x, y)]  # 座標リストの初期化

    elif event == cv2.EVENT_MOUSEMOVE:
        if drawing:
            cv2.line(image, (ix, iy), (x, y), (0, 255, 0), 2)
            ix, iy = x, y
            path.append((x, y))  # 座標を追加

    elif event == cv2.EVENT_LBUTTONUP:
        drawing = False
        cv2.line(image, (ix, iy), (x, y), (0, 255, 0), 2)
        path.append((x, y))  # 最終座標を追加

# カルマンフィルタの初期化
kalman = cv2.KalmanFilter(4, 2)
kalman.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
kalman.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)
kalman.processNoiseCov = np.eye(4, dtype=np.float32) * 0.03


# 空の画像を作成
# 注意：width, height, cv, timesはカルマン実行プログラムと同じ値に設定する。
width = 600
height = 600
cv = 80
times = 80 # 何ステップ分の移動かを指定する


image = np.zeros((width, height, 3), np.uint8)
# 研究室のマップを表示
map_coordinates = [((-1.5, -2), (3.2, -2)), # 通路拡大(一番下の線)
   ((-1.5, 0), (0.1, 0)),
   ((0.1, 0), (0.1, 2.4)),
   ((0.1, 2.4), (-1.5, 2.4)),
   ((-1.5, 3.2), (3.2, 3.2)),
   ((3.2, 3.2), (3.2, -0.0)), # 通路拡大
   ((3.2, 0), (2, 0)),
   ((2, 0), (2, 2.4)),
   ((2, 2.4), (3.2, 2.4))
]

for coord_pair in map_coordinates:
    pt1, pt2 = coord_pair

    # 座標を整数に変換してから線を引く
    pt1 = (int(pt1[0] * cv + width / 2), int(-1 * pt1[1] * cv + height / 2))  # 座標のスケーリング
    pt2 = (int(pt2[0] * cv + width / 2), int(-1 * pt2[1] * cv + height / 2))  # 座標のスケーリング
    cv2.line(image, pt1, pt2, (255, 255, 255), 2)

# 判定ライン
cv2.line(image, (int(0.1 * cv + width / 2), int(-1 * 0.3 * cv + height / 2)), (int(2 * cv + width / 2), int(-1 * 0.3 * cv + height / 2)), (100, 100, 255), 2)  # 3
cv2.line(image, (int(0.1 * cv + width / 2), int(-1 * 1.0 * cv + height / 2)), (int(2 * cv + width / 2), int(-1 * 1.0 * cv + height / 2)), (100, 255, 255), 2)  # 2
cv2.line(image, (int(0.1 * cv + width / 2), int(-1 * 1.7 * cv + height / 2)), (int(2 * cv + width / 2), int(-1 * 1.7 * cv + height / 2)), (255, 255, 100), 2)  # 1

cv2.namedWindow('Setting the travel route')
cv2.setMouseCallback('Setting the travel route', draw_path)

while True:
    cv2.imshow('Setting the travel route', image)
    key = cv2.waitKey(1) & 0xFF
    if key == 27:  # ESCキーで終了
        break

"""
# リアルタイム予測結果を表示
for point in path:
    measurement = np.array([[np.float32(point[0])], [np.float32(point[1])]])
    kalman.correct(measurement)
    prediction = kalman.predict()
    predict_pt = (int(prediction[0]), int(prediction[1]))
    cv2.circle(image, predict_pt, 5, (0, 0, 255), -1)
while True:
    cv2.imshow('image', image)
    
    key = cv2.waitKey(1) & 0xFF
    if key == 27:  # ESCキーで終了
        break

cv2.destroyAllWindows()
"""

#print(f"path:{path}\n")
print(f"pathの長さ:{len(path)}")

# ROSノードの初期化
rospy.init_node('path_publisher')
pub = rospy.Publisher('all_data_pc1', String, queue_size=10)
#pub = rospy.Publisher('man_made_data', String, queue_size=10)
rate = rospy.Rate(6)  # (1秒に●回)

# パスをパブリッシュ
#for i in range(0, len(path), 8):
print(f"飛ばす値：{len(path)//times}")
for i in range(0, len(path), len(path)//times):
    point = path[i]
    if i+len(path)//times <=  len(path):
        point_2 = path[i+len(path)//times]
    else:
        point_2 = point
    # 世界座標系に変換
    world_point_x = (point[0]-width/2) / float(cv)
    world_point_y = (point[1]-height/2) / float(cv) *(-1.0)
    radian = math.atan2(point_2[1] - point[1],point_2[0] - point[0])
    degree = radian * 180.0 / math.pi % 360
    now = datetime.now()
    current_time = now.strftime('%Y-%m-%d %H:%M:%S') + f".{now.microsecond:06d}"
    pub_msg = f"{current_time},0,{world_point_x},{world_point_y},{degree},0" # 時刻, ID(0), 世界座標X, Y, 角度(0), 距離(0)

    pub.publish(pub_msg)
    rospy.loginfo(f"publish: {pub_msg}")
    rate.sleep()

print("パブリッシュが完了しました。")
