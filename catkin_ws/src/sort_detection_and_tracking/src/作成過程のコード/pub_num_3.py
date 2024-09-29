#!/usr/bin/env python
# 1.16
# bbox.idを変更できるかテストするためのパブリッシャノード

import rospy
from std_msgs.msg import Int32

def number_publisher():
    # ノードの初期化
    rospy.init_node('number_publisher', anonymous=True)

    # パブリッシャの作成
    pub = rospy.Publisher('number_topic', Int32, queue_size=10)

    # パブリッシュする値
    number_to_publish = 3

    # 5秒待機
    rospy.sleep(5.0)

    # 一度だけパブリッシュする
    pub.publish(number_to_publish)
    rospy.loginfo(f"publish: {number_to_publish}")

    # ノードのシャットダウン
    rospy.spin()

if __name__ == '__main__':
    try:
        number_publisher()
    except rospy.ROSInterruptException:
        pass
