#!/usr/bin/env python
# 12.26
# sortの追跡結果を表示する
# IDを表示

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from darknet_ros_msgs.msg import BoundingBoxes

# from iou_tracker.msg import BoundingBoxes  # Import the BoundingBoxes message type
import cv2


class ImageSubscriber:
    def __init__(self):
        rospy.init_node("image_subscriber", anonymous=True)

        # イメージメッセージを受信するためのサブスクライバを作成
        self.image_sub = rospy.Subscriber(
            "/pc1/iou_tracker/detection_image", Image, self.image_callback
        )

        # BoundingBoxesメッセージを受信するためのサブスクライバを作成
        self.bounding_boxes_sub = rospy.Subscriber(
            "/pc1/iou_tracker/bounding_boxes",
            BoundingBoxes,
            self.bounding_boxes_callback,
        )

        # CvBridgeを初期化
        self.bridge = CvBridge()

        # bounding_boxesを格納するリスト
        self.bounding_boxes_list = []

    def bounding_boxes_callback(self, data):
        # BoundingBoxesメッセージを受信した際のコールバック
        self.bounding_boxes_list = data.bounding_boxes

    def image_callback(self, data):
        global img
        try:
            # イメージメッセージをOpenCVイメージに変換
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # bounding_boxes_listでfor文を回し、クラスが"person"の場合にidを取得してターミナルに表示
        count = 0  # 現在の表示人数確認用
        for box in self.bounding_boxes_list:
            if box.Class == "person":
                count += 1
                rospy.loginfo(f"[{count}] ID: {box.id}")
                cv2.putText(
                    img,
                    f"{box.id}",
                    (box.xmin + 20, box.ymin + 30),
                    cv2.FONT_HERSHEY_DUPLEX,
                    1.0,
                    (255, 0, 0),
                    2,
                )

        # OpenCVで画像を表示
        cv2.imshow("Detection Image", img)
        cv2.waitKey(3)  # キー入力があるまで待機

        rospy.loginfo(f"----------------------------------------------")

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    image_subscriber = ImageSubscriber()
    try:
        image_subscriber.run()
    except rospy.ROSInterruptException:
        pass
