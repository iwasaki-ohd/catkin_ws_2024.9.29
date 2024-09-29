#!/usr/bin/env python

import rospy
import sensor_msgs.msg
import numpy as np
from pcl import PointCloud
import open3d as o3d
from sensor_msgs import point_cloud2

def point_cloud_callback(msg):
    # 点群データを取得
    pc_data = np.array(list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))

    # Gaussian Splatting の実行
    splat_pc_data = gaussian_splatting(pc_data)

    # 点群データの視覚化
    visualize_point_cloud(splat_pc_data)

def gaussian_splatting(points, sigma=0.01):
    from scipy.spatial import KDTree

    kdtree = KDTree(points)
    splat_points = []
    for point in points:
        distances, indices = kdtree.query(point, k=10)
        weights = np.exp(-0.5 * (distances / sigma) ** 2)
        weighted_points = np.sum(weights[:, np.newaxis] * points[indices], axis=0) / np.sum(weights)
        splat_points.append(weighted_points)
    return np.array(splat_points)

def visualize_point_cloud(points):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    o3d.visualization.draw_geometries([pcd])

def listener():
    rospy.init_node('point_cloud_listener', anonymous=True)
    rospy.Subscriber("/camera/depth/color/points", sensor_msgs.msg.PointCloud2, point_cloud_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()

