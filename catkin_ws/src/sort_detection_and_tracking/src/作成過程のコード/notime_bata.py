#!/usr/bin/env python
# coding: UTF-8
# 1.12
# 2つのpcのデータを取得
# ID統合する
# 時間を考慮しない


import rospy
from std_msgs.msg import String
from collections import defaultdict
import datetime
import math

def callback(data, callback_args):
    topic_name, data_dict = callback_args
    rospy.loginfo("%s: %s", topic_name, data.data)

    # 以下のコードは変更なし
    # データの解析
    parsed_data = parse_data(data.data)
    #rospy.loginfo(f"parsed_data：{parsed_data}")

    
    # データを時刻順に格納
    timestamp = parsed_data[0]
    # rospy.loginfo(f"チェック：{parsed_data[1]}")
    data_dict[topic_name][timestamp] = parsed_data[1:] # data_dictはサブスクライブ数に応じてサイズが増していく
    #rospy.loginfo(f"チェック：{data_dict['pc1']}")

    
    # pc1とpc2のデータが揃った場合、処理を実行
    if len(data_dict['pc1']) > 0 and len(data_dict['pc2']) > 0:
        process_data(data_dict)

        # コールバック関数の処理が終わった後にdata_dictを初期化
        data_dict.clear()


def parse_data(data_str):
    # 文字列を解析してリストに変換
    data_list = data_str.split(', ')
    next_list = data_list[0].split(',')
    timestamp_str = next_list[0]
    timestamp = datetime.datetime.strptime(timestamp_str, "%Y-%m-%d %H:%M:%S.%f")
    rospy.loginfo(f"時間：{timestamp}")

    # IDをint型、世界座標をfloat型にして一つのリストに格納
    data = [int(next_list[i]) if i % 3 == 1 else float(next_list[i]) for i in range(1, len(next_list))]
    rospy.loginfo(f"data：{data}")
    return timestamp, data


def distance(coord1, coord2):
    # 2つの座標の距離を計算
    return math.sqrt((coord1[0] - coord2[0])**2 + (coord1[1] - coord2[1])**2)

def process_data(data_dict):
    pc1_data = data_dict['pc1']
    pc2_data = data_dict['pc2']

    rospy.loginfo(f"[確認用]pc1：{pc1_data}")

    # 時刻が最も近い物同士でマッチング
    time_diff = 0.0
    min_time_diff = float('inf')
    matched_data_pc1 = None
    matched_data_pc2 = None

    """
    # 先頭の要素を比較(マッチ判定)
    matched_data_pc1 = list(pc1_data.values())[0][0]
    matched_data_pc2 = list(pc2_data.values())[0][0]
    rospy.loginfo(f"マッチ：{matched_data_pc1}")
"""
    S = 0 # 時間差判定用
    # 毎回全データから最小時間のマッチを探している。あとで修正する
    for time_pc1, coord_pc1 in pc1_data.items():
        for time_pc2, coord_pc2 in pc2_data.items():
            time_diff = abs((time_pc1 - time_pc2).total_seconds()) # 時間差[秒]を計算
            rospy.loginfo(f"時間差：{time_diff}")
            if time_diff < min_time_diff and  time_diff <= 0.5:
                min_time_diff = time_diff
                matched_data_pc1 = coord_pc1[0]
                matched_data_pc2 = coord_pc2[0]
                S = 1
    if(S == 0):
        rospy.loginfo(f"データの時間差が大きいため、処理をスキップ。時間差：{time_diff}[秒]")

    else:
        rospy.loginfo(f"判定対象\n{matched_data_pc1} \n{matched_data_pc2}")
        # 半径0.3m以内にある場合、IDを同じものに変更
        if matched_data_pc1 is not None and matched_data_pc2 is not None:
            if len(matched_data_pc1) > 1 and len(matched_data_pc2) > 1:  # 要素が少なくとも2つ以上あることを確認
                # for 検出した人数分ループ

                if distance(matched_data_pc1[1:], matched_data_pc2[1:]) <= 0.3:
                    rospy.loginfo("Matching IDs: %d and %d", matched_data_pc1[0], matched_data_pc2[0])
                    matched_data_pc1[0] += 1000
                    matched_data_pc2[0] = matched_data_pc1[0]
                    rospy.loginfo("変更後の IDs: %d and %d", matched_data_pc1[0], matched_data_pc2[0])
                    rospy.loginfo(f"-----------------------------------------------------------")
                    
    rospy.loginfo(f"-----------------------------------------------------------")


def listener():
    rospy.init_node("listener", anonymous=True)

    # データを格納する辞書
    data_dict = defaultdict(dict)

    rospy.Subscriber("all_data_pc1", String, callback, callback_args=("pc1", data_dict))
    rospy.Subscriber("all_data_pc2", String, callback, callback_args=("pc2", data_dict))

    rospy.spin()

if __name__ == "__main__":
    listener()


