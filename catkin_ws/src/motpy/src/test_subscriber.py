#!/usr/bin/env python

import rospy
from std_msgs.msg import String


def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + " サブスクライブ： %s", data.data)
    # rospy.loginfo(" サブスクライブ： %s", data.data)

    # ---------------------------------------------------------------------------------
    # カンマごとに区切っってリストに格納
    multiple_data_list = data.data.split(", ")
    # リストの各要素を表示
    print(multiple_data_list)


def listener():
    rospy.init_node("listener", anonymous=True)

    rospy.Subscriber("multiple_data", String, callback)

    rospy.spin()


if __name__ == "__main__":
    listener()
