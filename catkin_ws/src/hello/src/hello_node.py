#!/usr/bin/env python
# coding: UTF-8

import rospy

def main():
    rospy.init_node("hello_node")

    rospy.loginfo("Hello World!")

    rospy.spin()


if __name__ == "__main__":
    main()