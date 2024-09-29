#!/usr/bin/env python
# coding: UTF-8

import rospy
from std_msgs.msg import Float32

# サブスクライブ時に呼ばれるコールバック関数
def on_subscribe(msg):
    # ログの出力
    rospy.loginfo("Depth data: {}".format(msg.data))

# メイン関数
def main():
    # ノードの初期化
    rospy.init_node("depth_data_subscriber")

    # サブスクライバの生成
    rospy.Subscriber("Depth_data", Float32, on_subscribe)

    # ノードを実行し、コールバック関数が呼び出されるまでスピン(ループ)する
    rospy.spin()

if __name__ == "__main__":
    main()
