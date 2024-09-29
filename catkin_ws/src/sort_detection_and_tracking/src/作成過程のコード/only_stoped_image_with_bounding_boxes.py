#!/usr/bin/env python
# 12.23
# sortの追跡結果を表示する

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class ImageSubscriber:
    def __init__(self):
        rospy.init_node('image_subscriber', anonymous=True)

        # イメージメッセージを受信するためのサブスクライバを作成
        self.image_sub = rospy.Subscriber('/iou_tracker/detection_image', Image, self.image_callback)
        #self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)

        # CvBridgeを初期化
        self.bridge = CvBridge()

    def image_callback(self, data):
        try:
            # イメージメッセージをOpenCVイメージに変換
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # OpenCVで画像を表示
        cv2.imshow('Detection Image', cv_image)
        cv2.waitKey(3)  # キー入力があるまで待機

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    image_subscriber = ImageSubscriber()
    try:
        image_subscriber.run()
    except rospy.ROSInterruptException:
        pass
