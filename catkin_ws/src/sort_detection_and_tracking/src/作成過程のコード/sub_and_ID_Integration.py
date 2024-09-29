#!/usr/bin/env python
# coding: UTF-8
# 1.9
# 2つのpcのデータを取得
# ID統合する

# 動かない

import rospy
from std_msgs.msg import String
from collections import defaultdict
import datetime
import math

def callback(data, topic_name, data_dict):
    rospy.loginfo("%s: %s", topic_name, data.data)

    # データの解析
    parsed_data = parse_data(data.data)

    # データを時刻順に格納
    timestamp = parsed_data[0]
    data_dict[topic_name][timestamp] = parsed_data[1:]

    # pc1とpc2のデータが揃った場合、処理を実行
    if len(data_dict['pc1']) > 0 and len(data_dict['pc2']) > 0:
        process_data(data_dict)

def parse_data(data_str):
    # 文字列を解析してリストに変換
    data_list = data_str.split(', ')
    timestamp = datetime.datetime.strptime(data_list[0], "%H:%M:%S.%f")
    data = [int(data_list[i]) if i % 4 == 1 else float(data_list[i]) for i in range(1, len(data_list))]
    return timestamp, data

def distance(coord1, coord2):
    # 2つの座標の距離を計算
    return math.sqrt((coord1[0] - coord2[0])**2 + (coord1[1] - coord2[1])**2)

def process_data(data_dict):
    pc1_data = data_dict['pc1']
    pc2_data = data_dict['pc2']

    # 時刻が最も近い物同士でマッチング
    min_time_diff = float('inf')
    matched_data_pc1 = None
    matched_data_pc2 = None

    for time_pc1, coord_pc1 in pc1_data.items():
        for time_pc2, coord_pc2 in pc2_data.items():
            time_diff = abs((time_pc1 - time_pc2).total_seconds())
            if time_diff < min_time_diff:
                min_time_diff = time_diff
                matched_data_pc1 = coord_pc1
                matched_data_pc2 = coord_pc2

    # 半径0.3m以内にある場合、IDを同じものに変更
    if matched_data_pc1 is not None and matched_data_pc2 is not None:
        if distance(matched_data_pc1[1:], matched_data_pc2[1:]) <= 0.3:
            rospy.loginfo("Matching IDs: %d and %d", matched_data_pc1[0], matched_data_pc2[0])

    # 処理が終わったデータを削除
    del pc1_data[min(pc1_data.keys())]
    del pc2_data[min(pc2_data.keys())]

def listener():
    rospy.init_node("listener", anonymous=True)

    # データを格納する辞書
    data_dict = defaultdict(dict)

    rospy.Subscriber("all_data_pc1", String, callback, callback_args=("pc1", data_dict))
    rospy.Subscriber("all_data_pc2", String, callback, callback_args=("pc2", data_dict))

    rospy.spin()

if __name__ == "__main__":
    listener()


