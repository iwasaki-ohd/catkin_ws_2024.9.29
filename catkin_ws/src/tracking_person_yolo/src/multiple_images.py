#!/usr/bin/env python
# 12.15
# 複数回保存できるように改良
# プログラム起動ごとにフォルダを作成し、そこに画像を保存する

import os
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import datetime

time_now = datetime.datetime.now()
# 作成するフォルダの名前を指定
folder_name = str(time_now)
# 画像の保存先ディレクトリ
SAVE_DIR = os.path.expanduser("~/saved_images" + "/" + folder_name)


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


def create_folder(folder_name):
    # フォルダを作成するパスを取得
    folder_path = os.path.join(os.path.expanduser("~/saved_images"), folder_name)

    # フォルダが存在しない場合は作成
    if not os.path.exists(folder_path):
        try:
            os.makedirs(folder_path)
            rospy.loginfo("フォルダ '{}' を作成しました。".format(folder_name))
        except OSError as e:
            rospy.logerr("フォルダの作成中にエラーが発生しました: {}".format(str(e)))
    else:
        rospy.logwarn("フォルダ '{}' は既に存在しています。".format(folder_name))


def save_image():
    global cv_image
    dt_now = datetime.datetime.now()
    file_name = "{}.png".format(dt_now)
    file_path = os.path.join(SAVE_DIR, file_name)
    cv2.imwrite(file_path, cv_image)
    rospy.loginfo("画像データをpngファイルに保存しました: {}".format(file_path))


def main():
    # 保存先ディレクトリが存在しない場合は作成
    # if not os.path.exists(SAVE_DIR):
    #   os.makedirs(SAVE_DIR)

    # ROSノードの初期化
    rospy.init_node("image_viewer", anonymous=True)
    # イメージサブスクライバの作成
    rospy.Subscriber("/camera/color/image_raw", Image, image_callback)

    rate = rospy.Rate(10)  # ループの実行頻度（10Hz）

    a = 0
    while not rospy.is_shutdown():
        if a == 0:
            # フォルダを作成
            create_folder(folder_name)
            a = 1

        # キー入力の待ち状態を作成
        user_input = input("Enter 'S' to save image, or 'Q' to quit: ")
        if user_input == "S":
            # Sが入力されたらデータを画像ファイルに保存
            save_image()
        elif user_input == "Q":
            # Qが入力されたら終了
            break

        rate.sleep()

    cv2.destroyAllWindows()  # ウィンドウを閉じる


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
