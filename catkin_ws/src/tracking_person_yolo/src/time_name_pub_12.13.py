#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import socket
from time import strftime


def publish_time_and_hostname():
    # ノードの初期化
    rospy.init_node("time_and_hostname_publisher", anonymous=True)

    # パブリッシャーの作成
    pub = rospy.Publisher("pc1_time_and_hostname", String, queue_size=10)

    # ループで繰り返し実行
    rate = rospy.Rate(5)  # 1Hz
    while not rospy.is_shutdown():
        # 現在の時間とPC名の取得
        current_time = strftime("%Y-%m-%d %H:%M:%S")
        pc_name = socket.gethostname()

        # メッセージの作成とパブリッシュ
        message = f"[PC1] Current Time: {current_time}, PC Name: {pc_name}"
        rospy.loginfo("\n" + message)
        # rospy.loginfo(type(message))
        pub.publish(message)

        # 次のイテレーションまで待機
        rate.sleep()


if __name__ == "__main__":
    try:
        publish_time_and_hostname()
    except rospy.ROSInterruptException:
        pass
