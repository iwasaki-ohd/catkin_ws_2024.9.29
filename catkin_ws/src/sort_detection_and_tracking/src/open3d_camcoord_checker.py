#!/usr/bin/env python3

import open3d as o3d
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray
import threading

# PLYファイルの読み込み
ply_file = "cam1_0_925.ply"  # 読み込みたいPLYファイルのパス
pcd = o3d.io.read_point_cloud(ply_file)

# 座標軸を表示するためのジオメトリを作成
axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=3.0, origin=[0, 0, 0])

# 円柱の共通設定
cylinder_radius = 0.1  # 円柱の半径
cylinder_height = 0.3  # 円柱の高さ

# グローバル変数でサブスクライブした位置を保持
current_position = [-1.476495385169983, 0.7417414784431458, 2.2947778701782227]  # 初期位置

# 点群表示を更新するための関数
def update_visualization(vis, cylinders):
    global current_position

    # 現在の位置に円柱を移動
    for cylinder in cylinders:
        cylinder.translate(current_position, relative=False)

    # 描画を更新
    vis.update_geometry(pcd)
    vis.update_geometry(axis)
    for cylinder in cylinders:
        vis.update_geometry(cylinder)
    
    vis.poll_events()
    vis.update_renderer()

# カメラ座標をサブスクライブするコールバック関数
def camera_coordinate_callback(msg):
    global current_position
    # サブスクライブしたデータから位置座標を取得
    current_position = msg.data[:3]  # 最初の3要素が位置座標

# ROSのサブスクライバー処理を別スレッドで動作させる
def ros_listener():
    rospy.Subscriber("/camera_coordinate", Float32MultiArray, camera_coordinate_callback)
    rospy.spin()

# Open3Dの視覚化とROSノードの並行処理を開始
def start_visualization():
    cylinders = []

    # 初期位置に円柱を1つ作成
    cylinder = o3d.geometry.TriangleMesh.create_cylinder(radius=cylinder_radius, height=cylinder_height)
    cylinder.paint_uniform_color([1, 1, 0])  # 円柱を黄色に設定
    cylinder.translate(current_position)  # 初期位置に移動
    cylinders.append(cylinder)

    # Open3Dのビジュアライゼーション設定
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Point Cloud with XYZ Axis and Cylinders", width=800, height=600)
    vis.add_geometry(pcd)
    vis.add_geometry(axis)
    for cylinder in cylinders:
        vis.add_geometry(cylinder)

    # ROSのサブスクライバースレッドを開始
    thread = threading.Thread(target=ros_listener)
    thread.start()

    # 描画ループ
    while True:
        update_visualization(vis, cylinders)

if __name__ == "__main__":
    # ROSノードをメインスレッドで初期化
    rospy.init_node('camera_coordinate_listener', anonymous=True)
    
    # Open3Dの視覚化を開始
    start_visualization()
