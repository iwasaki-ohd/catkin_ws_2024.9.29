#!/usr/bin/env python
# coding: UTF-8

import rospy
from std_msgs.msg import String

# メイン
def main():
    # ノードの生成
    rospy.init_node("my_publisher_node")

    #パブリッシャーの生成
    pub = rospy.Publisher("my_topic", String, queue_size=10)

    r = rospy.Rate(1) # 1Hz
    while not rospy.is_shutdown():
        # メッセージの生成
        msg = String()
        msg.data = "Hello World!"

        # メッセージのパブリッシュ
        pub.publish(msg)

        #ログの出力
        rospy.loginfo("Publish :" + msg.data)

        # スリープ
        r.sleep()




if __name__ == "__main__":
    main()