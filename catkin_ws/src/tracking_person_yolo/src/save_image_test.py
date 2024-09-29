#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


def image_callback(msg):
    try:
        # ROSイメージメッセージをOpenCVイメージに変換
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

        # 画像の表示
        cv2.imshow("Camera Image", cv_image)
        cv2.waitKey(1)

    except Exception as e:
        rospy.logerr("Error processing the image: {}".format(e))


def main():
    # ROSノードの初期化
    rospy.init_node("image_viewer", anonymous=True)

    # イメージサブスクライバの作成
    rospy.Subscriber("/camera/color/image_raw", Image, image_callback)

    # ウィンドウが閉じられるまでプログラムを実行
    rospy.spin()

    # ウィンドウを閉じる
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
