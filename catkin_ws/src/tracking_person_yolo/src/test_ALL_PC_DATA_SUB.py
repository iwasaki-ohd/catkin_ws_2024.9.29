#!/usr/bin/env python

import rospy
from std_msgs.msg import String  # String型のメッセージを使用するために必要


def callback(data, topic_name):
    # rospy.loginfo("start")
    # rospy.loginfo("Received data: %s", data.data)
    rospy.loginfo("Received data from topic %s: %s", topic_name, data.data)
    # rospy.loginfo("end")


def listener():
    rospy.init_node("listener", anonymous=True)

    # rospy.Subscriber("pc1_time_and_hostname", String, callback)
    # rospy.Subscriber("pc2_time_and_hostname", String, callback)
    rospy.Subscriber(
        "pc1_time_and_hostname", String, callback, callback_args="pc1_time_and_hostname"
    )
    rospy.Subscriber(
        "pc2_time_and_hostname", String, callback, callback_args="pc2_time_and_hostname"
    )
    rospy.spin()


if __name__ == "__main__":
    listener()
