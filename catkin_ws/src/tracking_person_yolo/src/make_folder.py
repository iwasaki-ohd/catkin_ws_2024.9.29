#!/usr/bin/env python
import rospy
import os

def create_folder(folder_name):
    # フォルダを作成するパスを取得
    folder_path = os.path.join(os.path.expanduser("~/画像の保存先"), folder_name)

    # フォルダが存在しない場合は作成
    if not os.path.exists(folder_path):
        try:
            os.makedirs(folder_path)
            rospy.loginfo("フォルダ '{}' を作成しました。".format(folder_name))
        except OSError as e:
            rospy.logerr("フォルダの作成中にエラーが発生しました: {}".format(str(e)))
    else:
        rospy.logwarn("フォルダ '{}' は既に存在しています。".format(folder_name))

if __name__ == '__main__':
    # ノードの初期化
    rospy.init_node('folder_creator_node', anonymous=True)

    # 作成するフォルダの名前を指定
    folder_name = "my_custom_folder"

    # フォルダを作成
    create_folder(folder_name)

    # ノードの実行を続ける
    rospy.spin()

