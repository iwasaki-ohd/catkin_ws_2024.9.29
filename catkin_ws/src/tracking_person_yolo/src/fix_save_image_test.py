#!/usr/bin/env python
# 12.14

# 画像一枚分でシャットダウンしてしまうので、次回はそれを改良する

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import datetime


def image_callback(msg):
    global cv_image
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

    rate = rospy.Rate(10)  # ループの実行頻度（10Hz）

    dt_now = datetime.datetime.now()
    file_name = "取得した画像_" + str(dt_now) + ".png"

    while not rospy.is_shutdown():
        user_input = input("Enter 'S' to save image: ")
        if user_input == "S":
            # Sが入力されたらデータを画像ファイルに保存
            cv2.imwrite(file_name, cv_image)
            rospy.loginfo("画像データをpngファイルに保存しました")

            break

        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
