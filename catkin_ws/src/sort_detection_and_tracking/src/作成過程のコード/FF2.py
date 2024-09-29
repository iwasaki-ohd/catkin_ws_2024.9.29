#!/usr/bin/env python
# coding: UTF-8
# 1.16
# 2つのpcのデータを取得
# ID統合する
# 時間差を考慮

# FFの改良版
# 二重マッチ防止
# 統合IDの保持

import rospy
from std_msgs.msg import String
from collections import defaultdict
import datetime
import math

# 統合済みのIDを格納するリスト
id_mappings_pc1 = []
id_mappings_pc2 = []

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
    rospy.loginfo(f"[確認用]pc2：{pc2_data}\n")

    # 時刻が最も近い物同士でマッチング
    time_diff = 0.0 # 時間差を格納
    min_time_diff = float('inf') # 最小時間を格納する変数
    allowable_time = 0.5 # 許容時間差[s]
    matched_data_pc1 = None
    matched_data_pc2 = None


    S = 0 # 時間差判定用
    # 蓄積データから時間差が最小な組み合わせを探す
    for time_pc1, coord_pc1 in pc1_data.items():
        for time_pc2, coord_pc2 in pc2_data.items():
            time_diff = abs((time_pc1 - time_pc2).total_seconds()) # 時間差[s]を計算
            rospy.loginfo(f"時間差：{time_diff}")
            if time_diff < min_time_diff and  time_diff <= allowable_time:
                min_time_diff = time_diff
                matched_data_pc1 = coord_pc1[0]
                matched_data_pc2 = coord_pc2[0]
                S = 1
    if(S == 0): # 許容時間差に収まるデータの組み合わせが見つからなかった場合
        rospy.loginfo(f"データの時間差が大きいため、処理をスキップ。時間差：{time_diff}[秒]")

    else:
        judge = True # ID統合の有無を判定する変数
        allowable_dist = 0.3 # 許容距離[m]
        rospy.loginfo(f"判定対象\n{matched_data_pc1} \n{matched_data_pc2}")
        # 統合IDと元のID、変更先IDをまとめるリストを作成
        
        # 許容距離以内にある場合、IDを同じものに変更
        if matched_data_pc1 is not None and matched_data_pc2 is not None:
            # 既に統合済みのIDであった場合、そのIDに変更
            for i in range(0, int(len(matched_data_pc1)/3)):# 検出した人数分ループ
                rospy.loginfo(f"id_mappings_pc1：{id_mappings_pc1}")
                for id_mapping in id_mappings_pc1:
                    original_id, merged_id = id_mapping
                    if matched_data_pc1[i * 3] == original_id:
                        rospy.loginfo(f"pc1：ID {original_id} は既に統合されています。統合後のID {merged_id} に変更します。")
                        matched_data_pc1[i * 3] = merged_id
            for j in range(0, int(len(matched_data_pc2)/3)):# 検出した人数分ループ
                for id_mapping in id_mappings_pc2:
                    original_id, merged_id = id_mapping
                    if matched_data_pc2[j * 3] == original_id:
                        rospy.loginfo(f"pc2：ID {original_id} は既に統合されています。統合後のID {merged_id} に変更します。")
                        matched_data_pc2[j * 3] = merged_id

            # 統合対象の探索
            for i in range(0, int(len(matched_data_pc1)/3)):# 検出した人数分ループ
                change_num_pc1 = None # IDを変更するデータの添字(i)を格納
                change_num_pc2 = None # IDを変更するデータの添字(j)を格納
                min_distance = float('inf')  # 最短距離を格納する変数

                for j in range(0, int(len(matched_data_pc2)/3)):# 検出した人数分ループ
                    # 両者(二人)の距離を求める(世界座標x-y平面上)
                    current_distance = distance(matched_data_pc1[i * 3 + 1: i * 3 + 2 + 1],
                                                matched_data_pc2[j * 3 + 1: j * 3 + 2 + 1])
                    current_distance = float(format(current_distance, '.3f')) # 小数第三位まで。floatに変換しないとエラーが起こる。
                    rospy.loginfo(f"ループ{i}-{j}　　比較対象：{matched_data_pc1[i * 3]}と{matched_data_pc2[j * 3]}　　2点間の距離：{current_distance}[m]")
                    if current_distance <= allowable_dist and current_distance < min_distance:
                        min_distance = current_distance
                        change_num_pc1 = i * 3
                        change_num_pc2 = j * 3
                # iのループが一周するごとにIDを統合
                if change_num_pc1 is not None and change_num_pc2 is not None:
                    rospy.loginfo("＜ID統合＞ \n                            Matching IDs: %d and %d", matched_data_pc1[change_num_pc1], matched_data_pc2[change_num_pc2])
                    rospy.loginfo(f"マッチしたペアの距離：{min_distance}[m]")
                    matched_data_pc1[change_num_pc1] += 100
                    pc2_before_id = matched_data_pc2[change_num_pc2]
                    matched_data_pc2[change_num_pc2] = matched_data_pc1[change_num_pc1]
                    rospy.loginfo("変更後の IDs: %d and %d", matched_data_pc1[change_num_pc1], matched_data_pc2[change_num_pc2])
                    rospy.loginfo(f"-----------------------------------------------------------")
                    judge = False

                    # 統合したIDのもとのIDと変更先IDをまとめるリストに追加
                    id_mappings_pc1.append((matched_data_pc1[change_num_pc1] - 100, matched_data_pc1[change_num_pc1]))
                    id_mappings_pc2.append((pc2_before_id, matched_data_pc2[change_num_pc2]))
                    # マッチした要素(人間)をリストから除く。二重マッチ防止。
                    del matched_data_pc2[change_num_pc2:change_num_pc2+3]
        if judge:
            rospy.loginfo(f"ID統合はなし")

        rospy.loginfo(f"統合したIDの元のIDと変更先ID(pc1): {id_mappings_pc1}")
        rospy.loginfo(f"統合したIDの元のIDと変更先ID(pc2): {id_mappings_pc2}")
        rospy.loginfo(f"-----------------------------------------------------------\n")


def listener():
    rospy.init_node("listener", anonymous=True)

    # データを格納する辞書
    data_dict = defaultdict(dict)

    rospy.Subscriber("all_data_pc1", String, callback, callback_args=("pc1", data_dict))
    rospy.Subscriber("all_data_pc2", String, callback, callback_args=("pc2", data_dict))

    rospy.spin()

if __name__ == "__main__":
    listener()


