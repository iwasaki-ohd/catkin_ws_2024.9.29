#!/usr/bin/env python
import rospy
import numpy as np
import datetime
from std_msgs.msg import Float32MultiArray

AAA = []

def data_callback(data):
    # サブスクライブしたデータをAAAに追加
    AAA.append(data.data)

def main():
    rospy.init_node('data_subscriber_node', anonymous=True)

    # data_X トピックのサブスクライバを作成
    rospy.Subscriber("world_point", Float32MultiArray, data_callback)

    rate = rospy.Rate(10)  # ループの実行頻度（10Hz）

    dt_now = datetime.datetime.now()
    file_name = "取得した座標データ_" + str(dt_now) + "_out.csv"

    while not rospy.is_shutdown():
        user_input = input("Enter 'Y' to save data: ")
        if user_input == 'Y':
            # Yが入力されたらデータをCSVファイルに保存
            np.savetxt(file_name, AAA, delimiter=",")
            print("データをCSVファイルに保存しました")

            break

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
