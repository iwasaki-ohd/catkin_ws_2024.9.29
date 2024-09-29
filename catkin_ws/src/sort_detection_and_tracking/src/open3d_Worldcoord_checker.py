#!/usr/bin/env python3

import open3d as o3d
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray
import threading
import math

# 初期パラメータ
initial_parameters = [
[-1.83697020e-16,  0.00000000e+00, -1.00000000e+00,  1.00000000e+00],
[ 1.00000000e+00, -1.83697020e-16, -1.83697020e-16,  1.00000000e+00],
[-1.83697020e-16, -1.00000000e+00,  3.37445951e-32,  1.00000000e+00],
[ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]
]
"""
    [0.0,  -1.0, 0.0, -1.0],
    [ 0.0, 0.0, -1.0, 1.0],
    [1.0, 0.0, 0.0, -1.2],
    [ 0.0, 0.0, 0.0, 1.0]
"""

"""
    [-1.83697020e-16,  0.00000000e+00, -1.00000000e+00, -1.0],
    [ 9.87688341e-01, -1.56434465e-01, -1.81435405e-16, 1.0],
    [-1.56434465e-01, -9.87688341e-01,  2.87365450e-17, -1.2],
    [ 0.0, 0.0, 0.0, 1.0]
"""
# 外部パラメータ
extrinsic_parameters = [
[ 0.8145467,   0.19523254,  0.54625812, -0.62378679],
[-0.26814295,  0.96174366,  0.05611137, -0.4424984 ],
[-0.51440552, -0.19218059,  0.83573535, -0.36722294],
[ 0.        ,  0.        ,  0.        ,  1.        ]
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
cylinder_height = 3  # 円柱の高さ

# グローバル変数でサブスクライブした位置を保持
current_position = [0, 0, 0]  # 初期位置
transformed_position = [-0.95, 0.9, 0]  # 変換された位置を保持するリストを初期化

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

    map_position = final_result[:3].flatten().tolist()

    world_position = map_position[:] # コピー元を参照しない
    #world_position[0] = (map_position[0]-0.95)
    #world_position[1] = (map_position[1]+0.9)

    #world_position[0] = (map_position[0]-0.95)*math.cos(math.radians(109)) + (map_position[1]+0.9)*(math.sin(math.radians(109)))
    #world_position[1] = -(map_position[0]-0.95)*math.sin(math.radians(109)) + (map_position[1]+0.9)*math.cos(math.radians(109))
    #world_position[0] = (map_position[0])*math.cos(math.radians(180)) + (map_position[1])*(-math.sin(math.radians(180)))
    #world_position[1] = (map_position[0])*math.sin(math.radians(180)) + (map_position[1])*math.cos(math.radians(180))
    
    rospy.loginfo(f"回転後のworld_position: \n{world_position}")


    # 一時テスト用
    #new_position = intermediate_result[:3].flatten().tolist()
    return world_position


# 点群表示を更新するための関数
def update_visualization(vis, cylinders):
    global transformed_position

    # 現在の位置に円柱を移動
    for cylinder in cylinders:
        #rospy.loginfo(f"最終座標　transformed_position: \n{transformed_position}")
        cylinder.translate(transformed_position, relative=False)

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
    #transformed_position = transform_position(current_position)
    transformed_position = transform_position(current_position)
    #print("変換された位置:", transformed_position)

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
    #transformed_position = transform_position(current_position)  # 初期変換位置を取得
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
