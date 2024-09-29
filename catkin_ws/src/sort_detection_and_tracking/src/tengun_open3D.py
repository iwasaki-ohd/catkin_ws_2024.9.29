#!/usr/bin/env python3
# -*- coding: utf-8 -*-

""""
import open3d as o3d

# 点群データの読み込み
pcd = o3d.io.read_point_cloud("cloud_3Dmap_6.26.ply")

# 点群の可視化
o3d.visualization.draw_geometries([pcd])

"""

import open3d as o3d

# 点群データの読み込み
pcd = o3d.io.read_point_cloud("cloud_3Dmap_6.26.ply")

# ダウンサンプリング
downpcd = pcd.voxel_down_sample(voxel_size=0.05)

# 法線推定
downpcd.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

# 結果の可視化
o3d.visualization.draw_geometries([downpcd])