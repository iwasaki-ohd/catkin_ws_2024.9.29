#!/usr/bin/env python
# coding: UTF-8

import rospy
from std_msgs.msg import String

# サブスクライブ時に呼ばれる
def on_subscribe(msg):
    # ログの出力
    rospy.loginfo("Subscriber :" + msg.data)


# メイン
def main():
    # ノードの生成
    rospy.init_node("my_subscriber_node")

    # サブスクライバーの生成
    pub = rospy.Subscriber("my_topic", String, on_subscribe)

    # ノード終了まで待機
    rospy.spin()



if __name__ == "__main__":
    main()