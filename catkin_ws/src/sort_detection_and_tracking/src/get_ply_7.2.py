#!/usr/bin/env python

# 背景切り取り機能がある(8.4)


import rospy
import pyrealsense2 as rs
import numpy as np
import cv2
from open3d import *
from sensor_msgs.msg import Image as ROSImage, PointCloud2
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header

# Initialize ROS node
rospy.init_node('realsense_pointcloud_node', anonymous=True)

# Publishers
image_pub = rospy.Publisher('/camera/color/image_raw', ROSImage, queue_size=10)
pointcloud_pub = rospy.Publisher('/camera/depth/points', PointCloud2, queue_size=10)
bridge = CvBridge()

# Create a pipeline
pipeline = rs.pipeline()

# Create a config and configure the pipeline to stream different resolutions of color and depth streams
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
profile = pipeline.start(config)

# Getting the depth sensor's depth scale
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
rospy.loginfo("Depth Scale is: %f", depth_scale)

# We will be removing the background of objects more than clipping_distance_in_meters meters away
clipping_distance_in_meters = 1  # 1 meter
clipping_distance = clipping_distance_in_meters / depth_scale

# Create an align object
align_to = rs.stream.color
align = rs.align(align_to)

# Getting camera intrinsics
intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
pinhole_camera_intrinsic = camera.PinholeCameraIntrinsic(intr.width, intr.height, intr.fx, intr.fy, intr.ppx, intr.ppy)

# Streaming loop
num = 0

def publish_images(color_image, depth_image):
    color_msg = bridge.cv2_to_imgmsg(color_image, "bgr8")
    depth_msg = bridge.cv2_to_imgmsg(depth_image, "passthrough")
    image_pub.publish(color_msg)

def publish_pointcloud(pcd):
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'camera_link'
    points = np.asarray(pcd.points)
    if len(points) > 0:
        pointcloud = pc2.create_cloud_xyz32(header, points)
        pointcloud_pub.publish(pointcloud)

try:
    while not rospy.is_shutdown():
        # Get frameset of color and depth
        frames = pipeline.wait_for_frames()

        # Align the depth frame to color frame
        aligned_frames = align.process(frames)

        # Get aligned frames
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        # Validate that both frames are valid
        if not depth_frame or not color_frame:
            continue

        color_image = np.asanyarray(color_frame.get_data())
        color = geometry.Image(color_image)

        depth_image = np.asanyarray(depth_frame.get_data())
        depth_image = (depth_image < clipping_distance) * depth_image
        depth = geometry.Image(depth_image)

        # Remove background - Set pixels further than clipping_distance to grey
        grey_color = 153
        depth_image_3d = np.dstack((depth_image, depth_image, depth_image))  # depth image is 1 channel, color is 3 channels
        # 背景抜き取りを一時コメントアウト(7/2)
        bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)

        # Generate the pointcloud and texture mappings
        rgbd = geometry.RGBDImage.create_from_color_and_depth(color, depth, convert_rgb_to_intensity=False)
        pcd = geometry.PointCloud.create_from_rgbd_image(rgbd, pinhole_camera_intrinsic)

        # Publish images and pointcloud
        publish_images(color_image, depth_image)
        publish_pointcloud(pcd)

        # Render images for visualization (optional)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        images = np.hstack((bg_removed, depth_colormap))
        #images = np.hstack((color_image, depth_colormap))
        cv2.namedWindow('aligned_frame', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('aligned_frame', images)
        key = cv2.waitKey(1)
        # Press 's' to save the point cloud
        if key & 0xFF == ord('s'):
            rospy.loginfo("Saving to {0}.ply...".format(num))
            io.write_point_cloud('{0}.ply'.format(num), pcd)
            rospy.loginfo("Done")
            num += 1
        # Press esc or 'q' to close the image window
        elif key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break
finally:
    pipeline.stop()

