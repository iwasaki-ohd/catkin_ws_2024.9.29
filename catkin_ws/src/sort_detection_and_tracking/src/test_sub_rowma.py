#!/usr/bin/env python
import rospy
from std_msgs.msg import String  # もしくは適切なメッセージ型を使用

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    rospy.loginfo(data.data)

def listener():
    rospy.init_node('listener', anonymous=True)

    # "/all_data_pc1" トピックに対してコールバック関数を登録
    rospy.Subscriber("/iwasaki_2/hello_pc2", String, callback)  # Stringの部分を適切なメッセージ型に変更

    # Ctrl+C が押されるまでスクリプトが終了しないようにする
    rospy.spin()

if __name__ == '__main__':
    listener()

