#!/usr/bin/env python
# coding: UTF-8
# 2024.4.17~
# robomech原稿提出時点(3.1)でのデータ統合プログラム
# 変数JGで二重パブリッシュを阻止。(pc3台以上には対応してない)
# 片方のpcデータのみを受け取ったときに、統合リストのIDとのIDチェックを行わない不具合を修正。
# 3.7
# トピック名の先頭にUUIDを追加

# 4.17
# 角度を含むデータを統合できるようにする。
# データ：時刻＋(ID, 世界座標X, 世界座標Y, 角度)

# 4.22
# 統合後のIDの基準を変更
# 先に検出された人物のIDを優先する
# ex. (1, 101)がマッチした時、101が先に検出されていたら統合後のIDを(101, 101)とする

# 4.26
# 距離もサブスクライブ
# # データ：時刻＋(ID, 世界座標X, 世界座標Y, 角度, 距離)

# 4.30
# カメラ3台に対応させる
# 変更箇所には「xxx」を残す
# 汎用性を上げるためにpc1, pc2をpcA, pcBのように記述する。
# 書き換えられない部分。
# id_mappings_pc1 、rospy.Subscriber("all_data_pc1", String, callback, callback_args=("pc1", data_dict))、data_dict['pc1']

# 5.1
# 33_Integrationの余分なコードを消したバージョン

# 5.3
# 同時三台分に対応

import rospy
from std_msgs.msg import String
from collections import defaultdict
import datetime
import math
import copy

# 統合済みのIDを格納するリスト。プログラムが終了するまで結果は保存される。
id_mappings_pc1 = []
id_mappings_pc2 = []
id_mappings_pc3 = []# xxx

max_data_dict_count = 5 # 辞書に保管する最大データ数

DetectionTime_dict = {} # 各人物の最初の検出時刻を保持する辞書

#detecting_num = 0 # 検出中の人数の合計(cam1, cam2, 重複する人物はカウント2)

def callback(data, callback_args):
    global JG
    pc_name, data_dict = callback_args
    rospy.loginfo("データを受信：%s: %s", pc_name, data.data)
    # 以下のコードは変更なし
    # データの解析
    detecting_num = 0 # 初期化
    # 連続する`,`を1つだけ取り除く処理
    cleaned_message = ','.join(filter(None, data.data.split(','))) # エラー回避用(たまに",,"のように、カンマが連続したメッセージが送られてきてしまうので対策)
    parsed_data = parse_data(cleaned_message)
    rospy.loginfo(f"parsed_data：{parsed_data}") # デバッグ 

    # データを時刻順に格納
    timestamp = parsed_data[0]
    data_dict[pc_name][timestamp] = parsed_data[1:] # data_dictはサブスクライブ数に応じてサイズが増していく
    rospy.loginfo(f"data_dict{pc_name}: {data_dict[pc_name]}")
    # 保存上限を超える場合、最も古いデータを削除
    if len(data_dict[pc_name]) > max_data_dict_count:
        oldest_timestamp = min(data_dict[pc_name].keys())
        rospy.loginfo(f"保存上限を超えました。最も古いデータを辞書から削除します。\n対象データの時間：{oldest_timestamp}\nデータ：{data_dict[pc_name][oldest_timestamp]}")
        del data_dict[pc_name][oldest_timestamp]
        
    #rospy.loginfo(f"確認：{parsed_data[1:]}")
    pc_data = parsed_data[1]# 片側のみのデータを受信した場合、この変数にデータを格納(時間を除く)。リスト型。
    #rospy.loginfo(f"確認しますA：pc_data = {pc_data}, id = {id(pc_data)}")
    
    # 最初の検出時刻を保持する
    for i in range(int(len(pc_data)/5)):
        person_id = pc_data[i * 5]
        if person_id not in DetectionTime_dict:
        # 未登録の場合、辞書に追加
            DetectionTime_dict[person_id] = timestamp
            #rospy.loginfo(f"人物ID {person_id} が検出されました。検出時刻: {timestamp}")
        else:
            #rospy.loginfo(f"人物ID {person_id} は既に検出されています。最初の検出時刻: {DetectionTime_dict[person_id]}")
            pass

    if JG == 0: # パブリッシュをまだ行っていない場合(1回の処理で複数回パブリッシュされないようにするための条件文)
        JG += 1
        # pc1とpc2のデータが揃った場合、処理を実行
        if (len(data_dict['pc1']) > 0 and len(data_dict['pc2']) > 0 and len(data_dict['pc3']) > 0): # xxx
            rospy.loginfo(f"JG:{JG}")
            # cam1-2, cam2-3でそれぞれ人物同定を行って、その結果を一つのメッセージとしてパブリッシュ
            # cam2のデータがダブらないように注意
            # cam1-2で統合されていないcam2の残り人物とcam3を比較する or それぞれで比較を行って、最後にString関数内に同じIDが２つ合ったら削除するようにする(時間も考慮すべき)
            process_data_3(data_dict['pc1'], id_mappings_pc1, data_dict['pc2'], id_mappings_pc2, data_dict['pc3'], id_mappings_pc3)
            # コールバック関数の処理が終わった後にdata_dictを初期化
            data_dict['pc1'].clear()
            data_dict['pc2'].clear()
            data_dict['pc3'].clear()
            rospy.loginfo(f"---------------------------処理終了-----------------------------------\n")
            
        elif (len(data_dict['pc1']) > 0 and len(data_dict['pc2']) > 0):
            #rospy.loginfo(f"JG:{JG}")
            process_data(data_dict['pc1'], id_mappings_pc1, data_dict['pc2'], id_mappings_pc2)
            # コールバック関数の処理が終わった後にdata_dictを初期化
            data_dict['pc1'].clear()
            data_dict['pc2'].clear()
            rospy.loginfo(f"---------------------------処理終了-----------------------------------\n")
        elif (len(data_dict['pc2']) > 0 and len(data_dict['pc3']) > 0):
            #rospy.loginfo(f"JG:{JG}")
            process_data(data_dict['pc2'], id_mappings_pc2, data_dict['pc3'], id_mappings_pc3)
            # コールバック関数の処理が終わった後にdata_dictを初期化
            data_dict['pc2'].clear()
            data_dict['pc3'].clear()
            rospy.loginfo(f"---------------------------処理終了-----------------------------------\n")            
        elif (len(data_dict['pc1']) > 0 and len(data_dict['pc3']) > 0):
            # 絶対にID統合されないが、関数を使い回せて楽なのでprocess_dataで処理を行う
            # 結果は必ず「統合なし」で、それぞれのカメラデータが一つの変数に格納されて(integration_data)そのままパブリッシュされる
            process_data(data_dict['pc1'], id_mappings_pc1, data_dict['pc3'], id_mappings_pc3)
            # コールバック関数の処理が終わった後にdata_dictを初期化
            data_dict['pc1'].clear()
            data_dict['pc3'].clear()
            #rospy.loginfo(f"確認しますB\npc_data = {pc_data}, id = {id(pc_data)}\npc_data = {pc_data}, id = {id(pc_data)}")
            rospy.loginfo(f"---------------------------処理終了-----------------------------------\n")

        elif len(data_dict['pc1']) > 0: # pc1のデータのみ受信した場合
            rospy.loginfo(f"受信したpc1のデータ：{pc_data}")
            rospy.loginfo(f"---統合済みIDの捜索---　　id_mappings_pc1：{id_mappings_pc1}")
            judgement_1 = 0 # 統合したかどうかの判定用変数
            for i in range(int(len(pc_data)/5)):# 検出した人数分ループ
                    for id_mapping in id_mappings_pc1:
                        original_id, merged_id = id_mapping
                        if pc_data[i * 5] == original_id:
                            rospy.loginfo(f"pc1：ID {original_id} は既に統合されています。統合後のID {merged_id} に変更します。")
                            pc_data[i * 5] = merged_id
                            judgement_1 += 1
            if judgement_1 > 0:
                rospy.loginfo(f"統合完了！ 統合後のデータ：{pc_data}")
            else:
                rospy.loginfo(f"統合はなし")
            str_pc_data = ""
            list_a = [val for index, val in enumerate(pc_data) if index % 5 != 4] # 距離データを取り除く
            for item in list_a:
                str_pc_data += str(item) + ","
            str_pc_data = str_pc_data[0:-1] # 先頭の"["と末尾の"],"を取り除く
            pub_integration_data.publish(str_pc_data)
            rospy.loginfo(f"{pc_name}のデータのみをパブリッシュ：{str_pc_data}")
            rospy.loginfo(f"---------------------------処理終了-----------------------------------\n")

        elif len(data_dict['pc2']) > 0: # pc2のデータのみ受信した場合
            rospy.loginfo(f"受信したpc2のデータ：{pc_data}")
            rospy.loginfo(f"---統合済みIDの捜索---　　id_mappings_pc2：{id_mappings_pc2}")
            #rospy.loginfo(f"id:{id(id_mappings_pc2)}")
            judgement_2 = 0 # 統合したかどうかの判定用変数
            #rospy.loginfo(f"デバッグ：{len(pc_data)/5}")
            for i in range(int(len(pc_data)/5)):# 検出した人数分ループ
                for id_mapping in id_mappings_pc2:
                    original_id, merged_id = id_mapping
                    #rospy.loginfo(f"デバッグ　　original_id:{original_id}、merged_id:{merged_id}")
                    if pc_data[i * 5] == original_id:
                        rospy.loginfo(f"pc2：ID {original_id} は既に統合されています。統合後のID {merged_id} に変更します。")
                        pc_data[i * 5] = merged_id
                        judgement_2 += 1
            if judgement_2 > 0:
                rospy.loginfo(f"統合完了！ 統合後のデータ：{pc_data}")
            else:
                rospy.loginfo(f"統合はなし")

            str_pc_data = ""
            list_a = [val for index, val in enumerate(pc_data) if index % 5 != 4] # 距離データを取り除く
            for item in list_a:
                str_pc_data += str(item) + ","
            str_pc_data = str_pc_data[0:-1] # 先頭の"["と末尾の"],"を取り除く
            pub_integration_data.publish(str_pc_data)
            rospy.loginfo(f"{pc_name}のデータのみをパブリッシュ：{str_pc_data}")
            rospy.loginfo(f"---------------------------処理終了-----------------------------------\n")

        elif len(data_dict['pc3']) > 0: # pc3のデータのみ受信した場合
            rospy.loginfo(f"受信したpc3のデータ：{pc_data}")
            rospy.loginfo(f"---統合済みIDの捜索---　　id_mappings_pc3：{id_mappings_pc3}")
            judgement_3 = 0 # 統合したかどうかの判定用変数
            #rospy.loginfo(f"デバッグ：{len(pc_data)/5}")
            for i in range(int(len(pc_data)/5)):# 検出した人数分ループ
                for id_mapping in id_mappings_pc3:
                    original_id, merged_id = id_mapping
                    #rospy.loginfo(f"デバッグ　　original_id:{original_id}、merged_id:{merged_id}")
                    if pc_data[i * 5] == original_id:
                        rospy.loginfo(f"pc3：ID {original_id} は既に統合されています。統合後のID {merged_id} に変更します。")
                        pc_data[i * 5] = merged_id
                        judgement_3 += 1
            if judgement_3 > 0:
                rospy.loginfo(f"統合完了！ 統合後のデータ：{pc_data}")
            else:
                rospy.loginfo(f"統合はなし")

            str_pc_data = ""
            list_a = [val for index, val in enumerate(pc_data) if index % 5 != 4] # 距離データを取り除く
            for item in list_a:
                str_pc_data += str(item) + ","
            str_pc_data = str_pc_data[0:-1] # 先頭の"["と末尾の"],"を取り除く
            pub_integration_data.publish(str_pc_data)
            rospy.loginfo(f"{pc_name}のデータのみをパブリッシュ：{str_pc_data}")
            rospy.loginfo(f"---------------------------処理終了-----------------------------------\n")

    else:
        JG = 0 # zzz

# 文字列を各要素に合った型に変換して、リストに追加
def parse_data(data_str):
    data_list = data_str.split(',')
    timestamp_str = data_list[0]
    timestamp = datetime.datetime.strptime(timestamp_str, "%Y-%m-%d %H:%M:%S.%f")
    # IDと角度をint型、世界座標と距離をfloat型にして一つのリストに格納
    data = [int(data_list[i]) if i % 5 == 1 or i % 5 == 4 else float(data_list[i]) for i in range(1, len(data_list))]

    return timestamp, data

# 2つの座標の距離を計算する
def distance(coord1, coord2):
    return math.sqrt((coord1[0] - coord2[0])**2 + (coord1[1] - coord2[1])**2)

# 人物同定を行う関数(比較対象データ数：２)
def process_data(data_dictA, id_mappingsA, data_dictB, id_mappingsB):
    global integration_data
    integration_data = "" # パブリッシュするデータをまとめる変数

    # ここでどのpcのデータを扱うかの条件文を記述する
    # あとで追加　xxx
    pcA_data = data_dictA
    pcB_data = data_dictB

    id_mappings_pcA = id_mappingsA
    id_mappings_pcB = id_mappingsB
    #rospy.loginfo(f"id:{id(id_mappings_pcB)}")

    # 時刻が最も近い物同士でマッチング
    time_diff = 0.0 # 時間差を格納
    min_time_diff = float('inf') # 最小時間を格納する変数
    allowable_time = 0.3 # 許容時間差[s]
    matched_data_pcA = None
    matched_data_pcB = None

    remained_data_pcA = None # IDを統合しなかった残りのデータを格納する
    del_num_list = [] # 今回マッチしたのデータのインデックスを保存するリスト(これを使用して、残りのデータremained_data_pcAの更新を行う)


    S = 0 # 時間差判定用
    # 蓄積データから時間差が最小な組み合わせを探す
    for time_pcA, coord_pcA in pcA_data.items():
        for time_pcB, coord_pcB in pcB_data.items():
            time_diff = abs((time_pcA - time_pcB).total_seconds()) # 時間差[s]を計算
            if time_diff < min_time_diff and  time_diff <= allowable_time:
                min_time_diff = time_diff
                matched_data_pcA = coord_pcA[0]
                matched_data_pcB = coord_pcB[0]
                S = 1

    if(S == 0): # 許容時間差に収まるデータの組み合わせが見つからなかった場合
        rospy.loginfo(f"データの時間差が大きいため、処理をスキップ。時間差：{time_diff}[秒]")


    else:
        judge = True # ID統合の有無を判定する変数
        allowable_dist = 0.5 # 許容距離[m]
        pcA_before_id = None # ID統合を行う場合に、統合前のIDを保持する変数
        pcB_before_id = None
        rospy.loginfo(f"判定対象データの時間差：{min_time_diff}\n                            判定対象データ\n                            {matched_data_pcA} \n                            {matched_data_pcB}")

        
        # 許容距離以内にある場合、IDを同じものに変更
        if matched_data_pcA is not None and matched_data_pcB is not None:
            # 既に統合済みのIDであった場合、そのIDに変更
            rospy.loginfo(f"統合済みIDリスト id_mappings_pcA：{id_mappings_pcA}")
            for i in range(int(len(matched_data_pcA)/5)):# 検出した人数分ループ
                for id_mapping in id_mappings_pcA:
                    original_id, merged_id = id_mapping
                    if matched_data_pcA[i * 5] == original_id:
                        rospy.loginfo(f"pcA：ID {original_id} は既に統合されています。統合後のID {merged_id} に変更します。")
                        matched_data_pcA[i * 5] = merged_id

            rospy.loginfo(f"統合済みIDリスト id_mappings_pcB：{id_mappings_pcB}")
            for j in range(int(len(matched_data_pcB)/5)):# 検出した人数分ループ
                for id_mapping in id_mappings_pcB:
                    original_id, merged_id = id_mapping
                    if matched_data_pcB[j * 5] == original_id:
                        rospy.loginfo(f"pcB：ID {original_id} は既に統合されています。統合後のID {merged_id} に変更します。")
                        matched_data_pcB[j * 5] = merged_id
            
            # 同じIDはデータリストから除く
            pcA_double_ID_index = [] # 同一のIDの添字を保管するリスト
            pcB_double_ID_index = []
            for i in range(int(len(matched_data_pcA)/5)):# 検出した人数分ループ
                for j in range(int(len(matched_data_pcB)/5)):# 検出した人数分ループ
                    if matched_data_pcA[i * 5] == matched_data_pcB[j * 5]:
                        pcA_double_ID_index.append(i*5) # IDを持つ添字:0, 5, 10, 15
                        pcB_double_ID_index.append(j*5)
                        rospy.loginfo(f"同一IDを発見。pcAのID:{matched_data_pcA[i * 5]}, pcBのID:{matched_data_pcB[j * 5]}")
                        integration_data += compare_dist(matched_data_pcA[i*5 :i*5 + 5], matched_data_pcB[j*5 :j*5 + 5])
            # IDを昇順に並べる
            sorted_pcA_del_index_list = sorted(pcA_double_ID_index, reverse=False)
            sorted_pcB_del_index_list = sorted(pcB_double_ID_index, reverse=False)
            for i in range(int(len(sorted_pcA_del_index_list))):
                if len(matched_data_pcA[(sorted_pcA_del_index_list[i] - i*5) : (sorted_pcA_del_index_list[i] - i*5 + 5)]) == 5: # 2024.2.29追加。エラー回避用
                    rospy.loginfo(f"pcA マッチングリストから削除：{matched_data_pcA[(sorted_pcA_del_index_list[i] - i*5)]}")
                    # xx
                    # 人間に近い方のカメラデータ(世界座標)を採用する
                    #integration_data += convert_to_string(matched_data_pcA[(sorted_pcA_del_index_list[i] - i*5) : (sorted_pcA_del_index_list[i] - i*5 + 4)]) # 距離を含まない人物情報をpub_dataに追加
                    del matched_data_pcA[(sorted_pcA_del_index_list[i] - i*5) : (sorted_pcA_del_index_list[i] - i*5 + 5)]
            for j in range(int(len(sorted_pcB_del_index_list))):
                rospy.loginfo(f"pcB マッチングリストから削除：{matched_data_pcB[(sorted_pcB_del_index_list[j] - j*5)]}")
                del matched_data_pcB[(sorted_pcB_del_index_list[j] - j*5) : (sorted_pcB_del_index_list[j] - j*5 + 5)]
                # xx



            
            remained_data_pcA = matched_data_pcA.copy()

            # 統合対象の探索
            for z in range(int(len(matched_data_pcA)/5)):# 検出した人数分ループ
                # rospy.loginfo(f"range : {range(int(len(matched_data_pcA)/4))}")
                change_num_pcA = None # IDを変更するデータの添字(i)を格納
                change_num_pcB = None # IDを変更するデータの添字(j)を格納
                min_distance = float('inf')  # 最短距離を格納する変数
                #rospy.loginfo(f"\nmatch_pc1: {matched_data_pcA}\nmatch_pc2: {matched_data_pcB}")
                for j in range(int(len(matched_data_pcB)/5)):# 検出した人数分ループ
                    # 両者(二人)の距離を求める(世界座標x-y平面上)
                    current_distance = distance(matched_data_pcA[z * 5 + 1: z * 5 + 2 + 1],
                                                matched_data_pcB[j * 5 + 1: j * 5 + 2 + 1])
                    current_distance = float(format(current_distance, '.3f')) # 小数第三位まで。floatに変換しないとエラーが起こる。
                    #rospy.loginfo(f"ループ{z}-{j}　　比較対象：{matched_data_pcA[z * 4]}と{matched_data_pcB[j * 4]}　　2点間の距離：{current_distance}[m]")
                    if current_distance <= allowable_dist and current_distance < min_distance:
                        min_distance = current_distance
                        change_num_pcA = z * 5
                        change_num_pcB = j * 5
                # zのループが一周するごとにIDを統合
                if change_num_pcA is not None and change_num_pcB is not None: 
                    if matched_data_pcA[change_num_pcA] != matched_data_pcB[change_num_pcB]:# マッチしたデータのIDが同一でない場合
                        #rospy.loginfo(f"\n\n\n\n\n")
                        rospy.loginfo("＜ID統合＞ \n                            Matching IDs: %d and %d", matched_data_pcA[change_num_pcA], matched_data_pcB[change_num_pcB])
                        rospy.loginfo(f"マッチしたペアの距離：{min_distance}[m]")

                        del_num_list.append(change_num_pcA)
                        #rospy.loginfo(f"del_num_list : {del_num_list}")
                        rospy.loginfo(f"辞書：{DetectionTime_dict}")
                        #detection_time_pcA = DetectionTime_dict[str(matched_data_pcA[change_num_pcA])] # 検出時刻の比較に使用
                        detection_time_pcA = DetectionTime_dict[matched_data_pcA[change_num_pcA]]
                        detection_time_pcB = DetectionTime_dict[matched_data_pcB[change_num_pcB]]

                        if(detection_time_pcA <= detection_time_pcB): # 先に検出された方のIDに統合
                            pcB_before_id = matched_data_pcB[change_num_pcB]
                            matched_data_pcB[change_num_pcB] = matched_data_pcA[change_num_pcA]
                            # 統合したIDのもとのIDと変更先IDをまとめるリストに追加
                            id_mappings_pcB.append((pcB_before_id, matched_data_pcB[change_num_pcB]))
                            # 距離データの比較
                            # カメラからの距離が近い方の世界座標を採用
                            #if matched_data_pcA[change_num_pcA+4] <= matched_data_pcB[change_num_pcB+4]:
                            integration_data += compare_dist(matched_data_pcA[change_num_pcA:change_num_pcA+5], matched_data_pcB[change_num_pcB:change_num_pcB+5])
                        else:
                            pcA_before_id = matched_data_pcA[change_num_pcA]
                            matched_data_pcA[change_num_pcA] = matched_data_pcB[change_num_pcB]
                            # 統合したIDのもとのIDと変更先IDをまとめるリストに追加
                            id_mappings_pcA.append((pcA_before_id, matched_data_pcA[change_num_pcA]))
                            integration_data += compare_dist(matched_data_pcA[change_num_pcA:change_num_pcA+5], matched_data_pcB[change_num_pcB:change_num_pcB+5])
                            #integration_data += convert_to_string(matched_data_pcA[change_num_pcA : change_num_pcA + 4])# cam1のデータを優先

                        rospy.loginfo("変更後の IDs: %d and %d", matched_data_pcA[change_num_pcA], matched_data_pcB[change_num_pcB])
                        rospy.loginfo(f"-----------------------------------------------------------")
                        judge = False

                        # マッチした要素(人間)をリストから除く。二重マッチ防止。
                        del matched_data_pcB[change_num_pcB:change_num_pcB+5]

            # IDを統合しなかった(それぞれのカメラでしか認識していない)人間のデータを追加
            #rospy.loginfo(f"del_num_list : {del_num_list}")
            if del_num_list != []:
                for i in range(int(len(del_num_list))):
                    del remained_data_pcA[(del_num_list[i] - i*5) : (del_num_list[i] - i*5 + 5)]
            # 残ったデータの追加
            for i in range(int(len(remained_data_pcA)/5)):
                integration_data += convert_to_string(remained_data_pcA[i*5:i*5+4])
            for j in range(int(len(matched_data_pcB)/5)):# 残った人数分ループ
                #rospy.loginfo(f"最終のmatch_pc2:{matched_data_pcB}")
                integration_data += convert_to_string(matched_data_pcB[j*5:j*5+4])
            
        if judge:
            rospy.loginfo(f"ID統合はなし")

        rospy.loginfo(f"pcAの統合済みIDリスト(元のIDと変更先ID): {id_mappings_pcA}")
        rospy.loginfo(f"pcBの統合済みIDリスト(元のIDと変更先ID): {id_mappings_pcB}")

        if integration_data != "":
            pub_integration_data.publish(integration_data[:-1]) # 文字列末尾の","を削除したものをパブリッシュ
            rospy.loginfo(f"パブリッシュ integration_data: {integration_data[:-1]}")

# 人物同定を行う関数(比較対象データ数：３)
def process_data_3(data_dictA, id_mappingsA, data_dictB, id_mappingsB, data_dictC, id_mappingsC):
    global integration_data
    integration_data = "" # パブリッシュするデータをまとめる変数

    # ここでどのpcのデータを扱うかの条件文を記述する
    # あとで追加　xxx
    pcA_data = data_dictA
    pcB_data = data_dictB
    pcC_data = data_dictC

    id_mappings_pcA = id_mappingsA
    id_mappings_pcB = id_mappingsB
    id_mappings_pcC = id_mappingsC
    #rospy.loginfo(f"id:{id(id_mappings_pcB)}")

    # 時刻が最も近い物同士でマッチング
    time_diff = 0.0 # 時間差を格納
    min_time_diff_AB = float('inf') # 最小時間を格納する変数
    min_time_diff_BC = float('inf') # 最小時間を格納する変数
    allowable_time = 0.3 # 許容時間差[s]
    matched_data_pcA = None
    matched_data_pcB = None
    matched_data_pcC = None

    remained_data_pcA = None # IDを統合しなかった残りのデータを格納する。
    remained_data_pcB = None # IDを統合しなかった残りのデータを格納する。B-CのID統合対象データとして使用。
    del_num_list_A = [] # 今回マッチしたデータのインデックスを保存するリスト(これを使用して、残りのデータremained_data_pcAの更新を行う)


    S = 0 # 時間差判定用
    matched_timeB = None # pcAとBの時間差比較でマッチしたpcBの時刻を保管する変数
    # 蓄積データから時間差が最小な組み合わせを探す
    # pcA:pcB
    for time_pcA, coord_pcA in pcA_data.items():
        for time_pcB, coord_pcB in pcB_data.items():
            time_diff = abs((time_pcA - time_pcB).total_seconds()) # 時間差[s]を計算
            if time_diff < min_time_diff_AB and  time_diff <= allowable_time:
                min_time_diff_AB = time_diff
                matched_data_pcA = coord_pcA[0]
                matched_data_pcB = coord_pcB[0]
                matched_timeB = time_pcB
                S = 2 # zzz

    # pcB:pcC
    if (S==0): # pcAとBでマッチしなかったら
        for time_pcB, coord_pcB in pcB_data.items():
            for time_pcC, coord_pcC in pcC_data.items():
                time_diff = abs((time_pcB - time_pcC).total_seconds()) # 時間差[s]を計算
                if time_diff < min_time_diff_BC and  time_diff <= allowable_time:
                    min_time_diff_BC = time_diff
                    matched_data_pcB = coord_pcB[0]
                    matched_data_pcC = coord_pcC[0]
                    S = 1 # zzz

    else: # pcAとBでマッチしていたら
        for time_pcC, coord_pcC in pcC_data.items():
            time_diff = abs((time_pcC - matched_timeB).total_seconds()) # 時間差[s]を計算
            if time_diff < min_time_diff_BC and  time_diff <= allowable_time:
                min_time_diff_BC = time_diff
                #matched_data_pcB = coord_pcB[0]
                matched_data_pcC = coord_pcC[0]
                rospy.loginfo(f"C:{matched_data_pcC}")
                S = 3 # zzz
    


    if(S == 0): # 許容時間差に収まるデータの組み合わせが見つからなかった場合
        rospy.loginfo(f"3つともデータの時間差が大きいため比較が行えません。処理を終了します。時間差：{time_diff}[秒]")

    elif(S==1):
        rospy.loginfo(f"ケース3：BとCのみマッチしました。pc2台分の処理(process_data)に移行します。")
        process_data(data_dictB, id_mappings_pcB, data_dictC, id_mappings_pcC)
        # 無駄な処理が増えるが、コードを書き直すのが面倒なので一旦放置
    elif(S==2):
        rospy.loginfo(f"ケース2：AとBのみマッチしました。pc2台分の処理(process_data)に移行します。")
        process_data(data_dictA, id_mappings_pcA, data_dictB, id_mappings_pcB)
        # 無駄な処理が増えるが、コードを書き直すのが面倒なので一旦放置
    elif(S==3):
        rospy.loginfo(f"ケース1：A-B,B-Cともにマッチしました。\n                            [1]A-Bの処理を開始します。")

        judge = True # ID統合の有無を判定する変数
        allowable_dist = 0.5 # 許容距離[m]
        pcA_before_id = None # ID統合を行う場合に、統合前のIDを保持する変数
        pcB_before_id = None
        rospy.loginfo(f"判定対象データの時間差：{min_time_diff_AB}\n                            判定対象データ\n                            {matched_data_pcA} \n                            {matched_data_pcB}")

        
        # 許容距離以内にある場合、IDを同じものに変更
        if matched_data_pcA is not None and matched_data_pcB is not None:
            # 既に統合済みのIDであった場合、そのIDに変更
            rospy.loginfo(f"統合済みIDリスト id_mappings_pcA：{id_mappings_pcA}")
            for i in range(int(len(matched_data_pcA)/5)):# 検出した人数分ループ
                for id_mapping in id_mappings_pcA:
                    original_id, merged_id = id_mapping
                    if matched_data_pcA[i * 5] == original_id:
                        rospy.loginfo(f"pcA：ID {original_id} は既に統合されています。統合後のID {merged_id} に変更します。")
                        matched_data_pcA[i * 5] = merged_id

            rospy.loginfo(f"統合済みIDリスト id_mappings_pcB：{id_mappings_pcB}")
            for j in range(int(len(matched_data_pcB)/5)):# 検出した人数分ループ
                for id_mapping in id_mappings_pcB:
                    original_id, merged_id = id_mapping
                    if matched_data_pcB[j * 5] == original_id:
                        rospy.loginfo(f"pcB：ID {original_id} は既に統合されています。統合後のID {merged_id} に変更します。")
                        matched_data_pcB[j * 5] = merged_id
            
            # 同じIDはデータリストから除く
            pcA_double_ID_index = [] # 同一のIDの添字を保管するリスト
            pcB_double_ID_index = []
            for i in range(int(len(matched_data_pcA)/5)):# 検出した人数分ループ
                for j in range(int(len(matched_data_pcB)/5)):# 検出した人数分ループ
                    if matched_data_pcA[i * 5] == matched_data_pcB[j * 5]:
                        pcA_double_ID_index.append(i*5) # IDを持つ添字:0, 5, 10, 15
                        pcB_double_ID_index.append(j*5)
                        rospy.loginfo(f"同一IDを発見。pcAのID:{matched_data_pcA[i * 5]}, pcBのID:{matched_data_pcB[j * 5]}")
                        integration_data += compare_dist(matched_data_pcA[i*5 :i*5 + 5], matched_data_pcB[j*5 :j*5 + 5])
            # IDを昇順に並べる
            sorted_pcA_del_index_list = sorted(pcA_double_ID_index, reverse=False)
            sorted_pcB_del_index_list = sorted(pcB_double_ID_index, reverse=False)
            for i in range(int(len(sorted_pcA_del_index_list))):
                if len(matched_data_pcA[(sorted_pcA_del_index_list[i] - i*5) : (sorted_pcA_del_index_list[i] - i*5 + 5)]) == 5: # 2024.2.29追加。エラー回避用
                    rospy.loginfo(f"pcA マッチングリストから削除：{matched_data_pcA[(sorted_pcA_del_index_list[i] - i*5)]}")
                    # xx
                    # 人間に近い方のカメラデータ(世界座標)を採用する
                    #integration_data += convert_to_string(matched_data_pcA[(sorted_pcA_del_index_list[i] - i*5) : (sorted_pcA_del_index_list[i] - i*5 + 4)]) # 距離を含まない人物情報をpub_dataに追加
                    del matched_data_pcA[(sorted_pcA_del_index_list[i] - i*5) : (sorted_pcA_del_index_list[i] - i*5 + 5)]
            for j in range(int(len(sorted_pcB_del_index_list))):
                rospy.loginfo(f"pcB マッチングリストから削除：{matched_data_pcB[(sorted_pcB_del_index_list[j] - j*5)]}")
                del matched_data_pcB[(sorted_pcB_del_index_list[j] - j*5) : (sorted_pcB_del_index_list[j] - j*5 + 5)]
                # xx

            
            remained_data_pcA = matched_data_pcA.copy()

            # 統合対象の探索
            for z in range(int(len(matched_data_pcA)/5)):# 検出した人数分ループ
                # rospy.loginfo(f"range : {range(int(len(matched_data_pcA)/4))}")
                change_num_pcA = None # IDを変更するデータの添字(i)を格納
                change_num_pcB = None # IDを変更するデータの添字(j)を格納
                min_distance = float('inf')  # 最短距離を格納する変数
                #rospy.loginfo(f"\nmatch_pc1: {matched_data_pcA}\nmatch_pc2: {matched_data_pcB}")
                for j in range(int(len(matched_data_pcB)/5)):# 検出した人数分ループ
                    # 両者(二人)の距離を求める(世界座標x-y平面上)
                    current_distance = distance(matched_data_pcA[z * 5 + 1: z * 5 + 2 + 1],
                                                matched_data_pcB[j * 5 + 1: j * 5 + 2 + 1])
                    current_distance = float(format(current_distance, '.3f')) # 小数第三位まで。floatに変換しないとエラーが起こる。
                    #rospy.loginfo(f"ループ{z}-{j}　　比較対象：{matched_data_pcA[z * 4]}と{matched_data_pcB[j * 4]}　　2点間の距離：{current_distance}[m]")
                    if current_distance <= allowable_dist and current_distance < min_distance:
                        min_distance = current_distance
                        change_num_pcA = z * 5
                        change_num_pcB = j * 5
                # zのループが一周するごとにIDを統合
                if change_num_pcA is not None and change_num_pcB is not None: 
                    if matched_data_pcA[change_num_pcA] != matched_data_pcB[change_num_pcB]:# マッチしたデータのIDが同一でない場合
                        #rospy.loginfo(f"\n\n\n\n\n")
                        rospy.loginfo("＜ID統合＞ \n                            Matching IDs: %d and %d", matched_data_pcA[change_num_pcA], matched_data_pcB[change_num_pcB])
                        rospy.loginfo(f"マッチしたペアの距離：{min_distance}[m]")

                        del_num_list_A.append(change_num_pcA)
                        #rospy.loginfo(f"del_num_list_A : {del_num_list_A}")
                        rospy.loginfo(f"辞書：{DetectionTime_dict}")
                        #detection_time_pcA = DetectionTime_dict[str(matched_data_pcA[change_num_pcA])] # 検出時刻の比較に使用
                        detection_time_pcA = DetectionTime_dict[matched_data_pcA[change_num_pcA]]
                        detection_time_pcB = DetectionTime_dict[matched_data_pcB[change_num_pcB]]

                        if(detection_time_pcA <= detection_time_pcB): # 先に検出された方のIDに統合
                            pcB_before_id = matched_data_pcB[change_num_pcB]
                            matched_data_pcB[change_num_pcB] = matched_data_pcA[change_num_pcA]
                            # 統合したIDのもとのIDと変更先IDをまとめるリストに追加
                            id_mappings_pcB.append((pcB_before_id, matched_data_pcB[change_num_pcB]))
                            # 距離データの比較
                            # カメラからの距離が近い方の世界座標を採用
                            #if matched_data_pcA[change_num_pcA+4] <= matched_data_pcB[change_num_pcB+4]:
                            integration_data += compare_dist(matched_data_pcA[change_num_pcA:change_num_pcA+5], matched_data_pcB[change_num_pcB:change_num_pcB+5])
                        else:
                            pcA_before_id = matched_data_pcA[change_num_pcA]
                            matched_data_pcA[change_num_pcA] = matched_data_pcB[change_num_pcB]
                            # 統合したIDのもとのIDと変更先IDをまとめるリストに追加
                            id_mappings_pcA.append((pcA_before_id, matched_data_pcA[change_num_pcA]))
                            integration_data += compare_dist(matched_data_pcA[change_num_pcA:change_num_pcA+5], matched_data_pcB[change_num_pcB:change_num_pcB+5])
                            #integration_data += convert_to_string(matched_data_pcA[change_num_pcA : change_num_pcA + 4])# cam1のデータを優先

                        rospy.loginfo("変更後の IDs: %d and %d", matched_data_pcA[change_num_pcA], matched_data_pcB[change_num_pcB])
                        rospy.loginfo(f"-----------------------------------------------------------")
                        judge = False

                        # マッチした要素(人間)をリストから除く。二重マッチ防止。
                        del matched_data_pcB[change_num_pcB:change_num_pcB+5]

            # IDを統合しなかった(それぞれのカメラでしか認識していない)人間のデータを追加
            #rospy.loginfo(f"del_num_list_A : {del_num_list_A}")
            if del_num_list_A != []:
                for i in range(int(len(del_num_list_A))):
                    del remained_data_pcA[(del_num_list_A[i] - i*5) : (del_num_list_A[i] - i*5 + 5)]
            # 残ったデータの追加
            for i in range(int(len(remained_data_pcA)/5)):
                integration_data += convert_to_string(remained_data_pcA[i*5:i*5+4])
            """
            for j in range(int(len(matched_data_pcB)/5)):# 残った人数分ループ
                #rospy.loginfo(f"最終のmatch_pc2:{matched_data_pcB}")
                integration_data += convert_to_string(matched_data_pcB[j*5:j*5+4])
            """

            remained_data_pcB = matched_data_pcB.copy() # A-Bの比較で統合されなかったデータ
            rospy.loginfo(f"pcBの未統合データ：{remained_data_pcB}")

        if judge:
            rospy.loginfo(f"ID統合はなし")

        rospy.loginfo(f"pcAの統合済みIDリスト(元のIDと変更先ID): {id_mappings_pcA}")
        rospy.loginfo(f"pcBの統合済みIDリスト(元のIDと変更先ID): {id_mappings_pcB}")

        if integration_data != "":
            #pub_integration_data.publish(integration_data[:-1]) # 文字列末尾の","を削除したものをパブリッシュ # cam2-3の処理の後にパブリッシュ
            #rospy.loginfo(f"パブリッシュ integration_data: {integration_data[:-1]}")
            rospy.loginfo(f"＜A-Bの処理終了＞ integration_data(暫定): {integration_data}")

        #---------------------ここからB-Cの処理------------------------------------
        
        judge = True # ID統合の有無を判定する変数
        pcB_before_id = None # ID統合を行う場合に、統合前のIDを保持する変数
        pcC_before_id = None
        del_num_list_B = [] # 今回マッチしたのデータのインデックスを保存するリスト(これを使用して、残りのデータremained_data_after_pcBの更新を行う)
        rospy.loginfo(f"判定対象データの時間差：{min_time_diff_BC}\n                            判定対象データ\n                            {remained_data_pcB} \n                            {matched_data_pcC}")

        
        # 許容距離以内にある場合、IDを同じものに変更
        if remained_data_pcB is not None and matched_data_pcC is not None:
            # 既に統合済みのIDであった場合、そのIDに変更
            # pcBのデータは既にID変更処理が終わっているのでスキップ。
            """
            rospy.loginfo(f"統合済みIDリスト id_mappings_pcB：{id_mappings_pcB}")
            for i in range(int(len(remained_data_pcB)/5)):# 検出した人数分ループ
                for id_mapping in id_mappings_pcB:
                    original_id, merged_id = id_mapping
                    if remained_data_pcB[i * 5] == original_id:
                        rospy.loginfo(f"pcB：ID {original_id} は既に統合されています。統合後のID {merged_id} に変更します。")
                        remained_data_pcB[i * 5] = merged_id
            """
            rospy.loginfo(f"統合済みIDリスト id_mappings_pcC：{id_mappings_pcC}")
            for j in range(int(len(matched_data_pcC)/5)):# 検出した人数分ループ
                for id_mapping in id_mappings_pcC:
                    original_id, merged_id = id_mapping
                    if matched_data_pcC[j * 5] == original_id:
                        rospy.loginfo(f"pcC：ID {original_id} は既に統合されています。統合後のID {merged_id} に変更します。")
                        matched_data_pcC[j * 5] = merged_id
            
            # 同じIDはデータリストから除く
            pcB_double_ID_index = [] # 同一のIDの添字を保管するリスト
            pcC_double_ID_index = []
            for i in range(int(len(remained_data_pcB)/5)):# 検出した人数分ループ
                for j in range(int(len(matched_data_pcC)/5)):# 検出した人数分ループ
                    if remained_data_pcB[i * 5] == matched_data_pcC[j * 5]:
                        pcB_double_ID_index.append(i*5) # IDを持つ添字:0, 5, 10, 15
                        pcC_double_ID_index.append(j*5)
                        rospy.loginfo(f"同一IDを発見。pcBのID:{remained_data_pcB[i * 5]}, pcCのID:{matched_data_pcC[j * 5]}")
                        integration_data += compare_dist(remained_data_pcB[i*5 :i*5 + 5], matched_data_pcC[j*5 :j*5 + 5])
            # IDを昇順に並べる
            sorted_pcB_del_index_list = sorted(pcB_double_ID_index, reverse=False)
            sorted_pcC_del_index_list = sorted(pcC_double_ID_index, reverse=False)
            for i in range(int(len(sorted_pcB_del_index_list))):
                if len(remained_data_pcB[(sorted_pcB_del_index_list[i] - i*5) : (sorted_pcB_del_index_list[i] - i*5 + 5)]) == 5: # 2024.2.29追加。エラー回避用
                    rospy.loginfo(f"pcB マッチングリストから削除：{remained_data_pcB[(sorted_pcB_del_index_list[i] - i*5)]}")
                    # xx
                    # 人間に近い方のカメラデータ(世界座標)を採用する
                    #integration_data += convert_to_string(remained_data_pcB[(sorted_pcB_del_index_list[i] - i*5) : (sorted_pcB_del_index_list[i] - i*5 + 4)]) # 距離を含まない人物情報をpub_dataに追加
                    del remained_data_pcB[(sorted_pcB_del_index_list[i] - i*5) : (sorted_pcB_del_index_list[i] - i*5 + 5)]
            for j in range(int(len(sorted_pcC_del_index_list))):
                rospy.loginfo(f"pcC マッチングリストから削除：{matched_data_pcC[(sorted_pcC_del_index_list[j] - j*5)]}")
                del matched_data_pcC[(sorted_pcC_del_index_list[j] - j*5) : (sorted_pcC_del_index_list[j] - j*5 + 5)]
                # xx

            
            remained_data_after_pcB = remained_data_pcB.copy()

            # 統合対象の探索
            for z in range(int(len(remained_data_pcB)/5)):# 検出した人数分ループ
                # rospy.loginfo(f"range : {range(int(len(remained_data_pcB)/4))}")
                change_num_pcB = None # IDを変更するデータの添字(i)を格納
                change_num_pcC = None # IDを変更するデータの添字(j)を格納
                min_distance = float('inf')  # 最短距離を格納する変数
                #rospy.loginfo(f"\nmatch_pc1: {remained_data_pcB}\nmatch_pc2: {matched_data_pcC}")
                for j in range(int(len(matched_data_pcC)/5)):# 検出した人数分ループ
                    # 両者(二人)の距離を求める(世界座標x-y平面上)
                    current_distance = distance(remained_data_pcB[z * 5 + 1: z * 5 + 2 + 1],
                                                matched_data_pcC[j * 5 + 1: j * 5 + 2 + 1])
                    current_distance = float(format(current_distance, '.3f')) # 小数第三位まで。floatに変換しないとエラーが起こる。
                    #rospy.loginfo(f"ループ{z}-{j}　　比較対象：{remained_data_pcB[z * 4]}と{matched_data_pcC[j * 4]}　　2点間の距離：{current_distance}[m]")
                    if current_distance <= allowable_dist and current_distance < min_distance:
                        min_distance = current_distance
                        change_num_pcB = z * 5
                        change_num_pcC = j * 5
                # zのループが一周するごとにIDを統合
                if change_num_pcB is not None and change_num_pcC is not None: 
                    if remained_data_pcB[change_num_pcB] != matched_data_pcC[change_num_pcC]:# マッチしたデータのIDが同一でない場合
                        #rospy.loginfo(f"\n\n\n\n\n")
                        rospy.loginfo("＜ID統合＞ \n                            Matching IDs: %d and %d", remained_data_pcB[change_num_pcB], matched_data_pcC[change_num_pcC])
                        rospy.loginfo(f"マッチしたペアの距離：{min_distance}[m]")

                        del_num_list_B.append(change_num_pcB)
                        #rospy.loginfo(f"del_num_list_B : {del_num_list_B}")
                        rospy.loginfo(f"辞書：{DetectionTime_dict}")
                        #detection_time_pcB = DetectionTime_dict[str(remained_data_pcB[change_num_pcB])] # 検出時刻の比較に使用
                        detection_time_pcB = DetectionTime_dict[remained_data_pcB[change_num_pcB]]
                        detection_time_pcC = DetectionTime_dict[matched_data_pcC[change_num_pcC]]

                        if(detection_time_pcB <= detection_time_pcC): # 先に検出された方のIDに統合
                            pcC_before_id = matched_data_pcC[change_num_pcC]
                            matched_data_pcC[change_num_pcC] = remained_data_pcB[change_num_pcB]
                            # 統合したIDのもとのIDと変更先IDをまとめるリストに追加
                            id_mappings_pcC.append((pcC_before_id, matched_data_pcC[change_num_pcC]))
                            # 距離データの比較
                            # カメラからの距離が近い方の世界座標を採用
                            #if remained_data_pcB[change_num_pcB+4] <= matched_data_pcC[change_num_pcC+4]:
                            integration_data += compare_dist(remained_data_pcB[change_num_pcB:change_num_pcB+5], matched_data_pcC[change_num_pcC:change_num_pcC+5])
                        else:
                            pcB_before_id = remained_data_pcB[change_num_pcB]
                            remained_data_pcB[change_num_pcB] = matched_data_pcC[change_num_pcC]
                            # 統合したIDのもとのIDと変更先IDをまとめるリストに追加
                            id_mappings_pcB.append((pcB_before_id, remained_data_pcB[change_num_pcB]))
                            integration_data += compare_dist(remained_data_pcB[change_num_pcB:change_num_pcB+5], matched_data_pcC[change_num_pcC:change_num_pcC+5])

                        rospy.loginfo("変更後の IDs: %d and %d", remained_data_pcB[change_num_pcB], matched_data_pcC[change_num_pcC])
                        rospy.loginfo(f"-----------------------------------------------------------")
                        judge = False

                        # マッチした要素(人間)をリストから除く。二重マッチ防止。
                        del matched_data_pcC[change_num_pcC:change_num_pcC+5]

            # IDを統合しなかった(それぞれのカメラでしか認識していない)人間のデータを追加
            #rospy.loginfo(f"del_num_list_B : {del_num_list_B}")
            if del_num_list_B != []:
                for i in range(int(len(del_num_list_B))):
                    del remained_data_after_pcB[(del_num_list_B[i] - i*5) : (del_num_list_B[i] - i*5 + 5)]
            # 残ったデータの追加
            for i in range(int(len(remained_data_after_pcB)/5)):
                integration_data += convert_to_string(remained_data_after_pcB[i*5:i*5+4])
            
            for j in range(int(len(matched_data_pcC)/5)):# 残った人数分ループ
                #rospy.loginfo(f"最終のmatch_pc3:{matched_data_pcC}")
                integration_data += convert_to_string(matched_data_pcC[j*5:j*5+4])
            


        if judge:
            rospy.loginfo(f"ID統合はなし")

        rospy.loginfo(f"pcAの統合済みIDリスト(元のIDと変更先ID): {id_mappings_pcB}")
        rospy.loginfo(f"pcBの統合済みIDリスト(元のIDと変更先ID): {id_mappings_pcC}")

        if integration_data != "":
            pub_integration_data.publish(integration_data[:-1]) # 文字列末尾の","を削除したものをパブリッシュ # cam2-3の処理の後にパブリッシュ
            rospy.loginfo(f"＜B-Cの処理終了＞ パブリッシュ integration_data: {integration_data[:-1]}")


    else:
        rospy.logerr(f"処理不可能なデータです。")
        


# リストの内容をString型に変更する関数
def convert_to_string(lst):
    result = []
    
    # リストの長さが4の倍数であることを確認
    if len(lst) != 4:
        raise ValueError("リストの長さは4である必要があります")
    
    # 連続する4つの要素を取り出し、カンマで区切った文字列に変換
    for i in range(0, len(lst), 4):
        chunk = lst[i:i+4]
        chunk_str = ','.join(map(str, chunk)) + ","
        result.append(chunk_str)
    
    return result[0]

#距離を比較する
def compare_dist(lst1, lst2):
    result = []

    # リストの長さが5であることを確認
    if len(lst1) != 5 or len(lst2) != 5:
        raise ValueError("リストの長さは5である必要があります")
    if lst1[4] <= lst2[4]:
        #rospy.loginfo(f"確かめます：{lst1[4]}vs{lst2[4]}")
        result = lst1[0:4]
        #rospy.loginfo(f"チェック：{result}")
        # もし角度がcam1で取得できていなかったら、cam2のデータを使用する
        if result[3] == -1:
            rospy.loginfo(f"1_角度データを補填：変更前は{result[3]}、変更後は{lst2[3]}")
            result[3] = lst2[3]
        result = convert_to_string(result[0:4])
    else:
        result = lst2[0:4]
        #rospy.loginfo(f"チェック：{result}")
        # もし角度がcam2で取得できていなかったら、cam1のデータを使用する
        if result[3] == -1:
            rospy.loginfo(f"2_角度データを補填：変更前は{result[3]}、変更後は{lst1[3]}")
            result[3] = lst1[3]
        result = convert_to_string(result[0:4])

    #rospy.loginfo(f"結果を出力します：{result[:-1]}")    
    return result


def main():
    global pub_integration_data, JG
    rospy.init_node("Integrator", anonymous=True)

    # パブリッシャの作成
    pub_integration_data = rospy.Publisher("integration_data", String, queue_size=10)    

    # データを格納する辞書
    data_dict = defaultdict(dict)

    JG = 0 # パブリッシュ数調整用変数
    rospy.Subscriber("all_data_pc1", String, callback, callback_args=("pc1", data_dict))
    rospy.Subscriber("all_data_pc2", String, callback, callback_args=("pc2", data_dict))
    rospy.Subscriber("all_data_pc3", String, callback, callback_args=("pc3", data_dict)) # xxx
    #rospy.Subscriber("/iwasaki_2/all_data_pc2", String, callback, callback_args=("pc2", data_dict))
    #rospy.loginfo(f"{data_dict['pc1']}, {data_dict['pc2']}, {data_dict['pc3']}")
    #handler(data_dict)

    rospy.spin()

if __name__ == "__main__":
    main()


