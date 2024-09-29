#!/usr/bin/env python
# coding: UTF-8

import rospy
from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String, Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np


class RealSenseRosUnity:
    def __init__(self):
        self.depth_image = None
        self.bounding_boxes_list = []
        self.around = 3
        self.A_cord = np.array([[1.0], [1.0], [1.0], [1.0]])
        self.bridge = CvBridge()
        self.extrinsic_parameters = [
            [1.12540220e00, 1.14521609e-01, 2.14551412e-02, 4.41361242e-01],
            [1.02189019e-01, -2.12426572e-01, 9.93322660e-01, -1.74585221e00],
            [-1.52477111e-01, -1.22097908e00, -4.83816232e-01, 1.78920889e00],
            [3.88578059e-16, -3.33066907e-16, 2.22044605e-16, 1.00000000e00],
        ]
        self.K_fx = 0
        self.K_fy = 0
        self.K_cx = 0
        self.K_cy = 0
        self.image_w = 0
        self.image_h = 0

        rospy.init_node("realsense_ros_unity", anonymous=True)
        self.multiple_data_pub = rospy.Publisher("multiple_data", String, queue_size=10)
        self.world_point_pub = rospy.Publisher(
            "world_point", Float32MultiArray, queue_size=10
        )
        rospy.Subscriber(
            "/camera/aligned_depth_to_color/camera_info",
            CameraInfo,
            self.camera_info_callback,
        )
        rospy.Subscriber(
            "/camera/aligned_depth_to_color/image_raw", Image, self.depth_image_callback
        )
        rospy.Subscriber("/bounding_boxes", BoundingBoxes, self.bounding_boxes_callback)

    def depth_image_callback(self, data):
        CLmap = None
        self.depth_image = self.bridge.imgmsg_to_cv2(
            data, desired_encoding="passthrough"
        )

        if self.depth_image is not None:
            CLmap = cv2.applyColorMap(
                cv2.convertScaleAbs(self.depth_image, alpha=0.08), cv2.COLORMAP_JET
            )

        for face_box in self.bounding_boxes_list:
            if face_box.Class == "face":
                x_center = int((face_box.xmax + face_box.xmin) / 2)
                y_center = int((face_box.ymax + face_box.ymin) / 2)
                cv2.circle(
                    CLmap,
                    (x_center, y_center),
                    radius=self.around,
                    color=(255, 255, 255),
                    thickness=10,
                )

        cv2.imshow("Depth", CLmap)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            rospy.signal_shutdown("User initiated shutdown")

    def bounding_boxes_callback(self, data):
        self.bounding_boxes_list = data.bounding_boxes

        if self.depth_image is None:
            rospy.loginfo("depth_image is None. Skipping bounding_boxes_callback.")
            return

        rospy.loginfo("検出されたオブジェクトの数: {} [個] ".format(len(self.bounding_boxes_list)))
        count = 0
        multiple_data = ""

        for face_box in self.bounding_boxes_list:
            count += 1
            if face_box.Class == "face":
                x_center = int((face_box.xmax + face_box.xmin) / 2)
                y_center = int((face_box.ymax + face_box.ymin) / 2)

                rospy.loginfo(
                    "[{}]\nオブジェクトID:  {} \n2Dの中央座標(x, y): ({}, {})".format(
                        count, format(face_box.id, "x"), x_center, y_center
                    )
                )

                depth_values = self.depth_image[
                    y_center - self.around : y_center + self.around,
                    x_center - self.around : x_center + self.around,
                ]

                average_depth = np.mean(depth_values) / 1000

                formatted_average_depth = "{:.5f}".format(average_depth)
                rospy.loginfo(
                    "カメラとの平均距離: {} [m] ".format(float(formatted_average_depth))
                )

                coordinate3D_x = (
                    (x_center - self.image_w / 2) * average_depth / self.K_fx
                )
                coordinate3D_y = (
                    (y_center - self.image_h / 2) * average_depth / self.K_fy
                )

                rospy.loginfo(
                    "3D座標(x, y, z) =  ({}, {}, {})".format(
                        coordinate3D_x, coordinate3D_y, average_depth
                    )
                )

                camera_coordinate = [[coordinate3D_x, coordinate3D_y, average_depth]]
                self.A_cord[0:3, 0] = camera_coordinate[0]
                world_coordinate = (
                    np.dot(self.extrinsic_parameters, np.array(self.A_cord))[0:3]
                ).T
                rospy.loginfo(
                    "世界座標(x, y, z) =  ({}, {}, {})".format(
                        world_coordinate[0][0],
                        world_coordinate[0][1],
                        world_coordinate[0][2],
                    )
                )

                world_point = [
                    world_coordinate[0][0],
                    world_coordinate[0][1],
                    world_coordinate[0][2],
                ]
                world_point_msg = Float32MultiArray(data=world_point)
                self.world_point_pub.publish(world_point_msg)

                formatted_world_coordinate_x = "{:.5f}".format(world_coordinate[0][0])
                formatted_world_coordinate_y = "{:.5f}".format(world_coordinate[0][1])

                if count == 1:
                    multiple_data += (
                        str(format(face_box.id, "x"))
                        + (", ")
                        + str(formatted_world_coordinate_x)
                        + (", ")
                        + str(formatted_world_coordinate_y)
                    )
                else:
                    multiple_data += (
                        (", ")
                        + str(format(face_box.id, "x"))
                        + (", ")
                        + str(formatted_world_coordinate_x)
                        + (", ")
                        + str(formatted_world_coordinate_y)
                    )

        if len(multiple_data) > 2:
            self.multiple_data_pub.publish(multiple_data)

        rospy.loginfo("データをパブリッシュ: " + (multiple_data) + "\n ")
        rospy.loginfo("----------------------------------------")

    def camera_info_callback(self, msg):
        K = msg.K
        self.K_fx = K[0]
        self.K_fy = K[4]
        self.K_cx = K[2]
        self.K_cy = K[5]
        self.image_w = msg.width
        self.image_h = msg.height


def main():
    real_sense_ros_unity = RealSenseRosUnity()
    rospy.spin()


if __name__ == "__main__":
    main()
