#!/usr/bin/env python3

import open3d as o3d
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray
import threading

# 初期パラメータ
initial_parameters = [
    [0.0,  -1.0, 0.0, -1.0],
    [ 0.0, 0.0, -1.0, 1.0],
    [1.0, 0.0, 0.0, -1.2],
    [ 0.0, 0.0, 0.0, 1.0]
]
"""
    [-1.83697020e-16,  0.00000000e+00, -1.00000000e+00, -1.0],
    [ 9.87688341e-01, -1.56434465e-01, -1.81435405e-16, 1.0],
    [-1.56434465e-01, -9.87688341e-01,  2.87365450e-17, -1.2],
    [ 0.0, 0.0, 0.0, 1.0]
"""
# 外部パラメータ
extrinsic_parameters = [
    [ 0.81490274,  0.19735933,  0.54496129, -0.57892849],
    [-0.34787722,  0.91859486,  0.18752311, -0.01730745],
    [-0.46358921, -0.34239272,  0.81722229,  0.26584073],
    [ 0.0, 0.0, 0.0, 1.0]
]
print(initial_parameters)
print(extrinsic_parameters)

# PLYファイルの読み込み
ply_file = "map2_925.ply"  # 読み込みたいPLYファイルのパス
pcd = o3d.io.read_point_cloud(ply_file)

# 座標軸を表示するためのジオメトリを作成
axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=3.0, origin=[0, 0, 0])

# 円柱の共通設定
cylinder_radius = 0.1  # 円柱の半径
cylinder_height = 0.3  # 円柱の高さ

# グローバル変数でサブスクライブした位置を保持
current_position = [-1.476495385169983, 0.7417414784431458, 2.2947778701782227]  # 初期位置
transformed_position = []  # 変換された位置を保持するリストを初期化

def transform_position(current_position):
    # current_positionを同次座標に変換
    current_homogeneous = np.array(current_position + [1.0]).reshape(4, 1)
    rospy.loginfo(f"curremt: \n{current_homogeneous}")

    # initial_parametersとextrinsic_parametersをNumPy配列に変換
    initial_matrix = np.array(initial_parameters)
    extrinsic_matrix = np.array(extrinsic_parameters)
    # 行列積を計算 (np.dotを使用)
    intermediate_result = np.dot(initial_matrix, current_homogeneous)
    rospy.loginfo(f"intermediate_result: \n{intermediate_result}")
    final_result = np.dot(extrinsic_matrix, intermediate_result)
    rospy.loginfo(f"final_result: \n{final_result}")

    # 最初の3つの値を取り出して新しい座標とする
    new_position = final_result[:3].flatten().tolist()
    rospy.loginfo(f"new_position: \n{new_position}")
    # 一時テスト用
    #new_position = intermediate_result[:3].flatten().tolist()
    return new_position


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
    global transformed_position
    # サブスクライブしたデータから位置座標をリストとして取得
    current_position = list(msg.data[:3])  # 最初の3要素が位置座標をリストに変換

    # 現在の位置を変換
    transformed_position = transform_position(current_position)
    print("変換された位置:", transformed_position)

# ROSのサブスクライバー処理を別スレッドで動作させる
def ros_listener():
    rospy.Subscriber("/camera_coordinate", Float32MultiArray, camera_coordinate_callback)
    rospy.spin()

# Open3Dの視覚化とROSノードの並行処理を開始
def start_visualization():
    global transformed_position
    cylinders = []

    # 初期位置に円柱を1つ作成
    cylinder = o3d.geometry.TriangleMesh.create_cylinder(radius=cylinder_radius, height=cylinder_height)
    cylinder.paint_uniform_color([1, 1, 0])  # 円柱を黄色に設定
    transformed_position = transform_position(current_position)  # 初期変換位置を取得
    cylinder.translate(transformed_position)  # 初期位置に移動
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
