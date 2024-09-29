#!/usr/bin/env python

import rospy
from darknet_ros_msgs.msg import BoundingBoxes

def bounding_boxes_callback(data):
    for box in data.bounding_boxes:
        if box.Class == "person":
            # バウンディングボックスの中央座標を計算する
            x_center = (box.xmin + box.xmax) / 2
            y_center = (box.ymin + box.ymax) / 2
            # ターミナルに中央座標を表示する
            rospy.loginfo("人が検出されました。中央座標 (x, y): ({}, {})".format(x_center, y_center))

def main():
    # ノードの初期化
    rospy.init_node('person_detection', anonymous=True)
    # BoundingBoxesメッセージのサブスクライブ
    rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, bounding_boxes_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
