#!/usr/bin/env python

import rospy
from std_msgs.msg import String  # String型のメッセージを使用するために必要


def callback(data, topic_name):
    rospy.loginfo("%s: %s", topic_name, data.data)


def listener():
    rospy.init_node("listener", anonymous=True)

    rospy.Subscriber("all_data_pc1", String, callback, callback_args="pc1")
    rospy.Subscriber("all_data_pc2", String, callback, callback_args="pc2")

    rospy.spin()


if __name__ == "__main__":
    listener()
