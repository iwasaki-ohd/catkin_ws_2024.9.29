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
# ex) (1, 101)がマッチした時、101が先に検出されていたら統合後のIDを(101, 101)とする

# 4.26
# 距離もサブスクライブ
# # データ：時刻＋(ID, 世界座標X, 世界座標Y, 角度, 距離)

# 4.30
# カメラ3台に対応させる
# 変更箇所には「xxx」を残す

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
        # pc1とpc2のデータが揃った場合、処理を実行
        if (len(data_dict['pc1']) > 0 and len(data_dict['pc2']) > 0) or \
            (len(data_dict['pc1']) > 0 and len(data_dict['pc3']) > 0) or \
            (len(data_dict['pc2']) > 0 and len(data_dict['pc3']) > 0): # カメラデータが2つ以上揃ったら処理開始　xxx
            JG += 1
            #rospy.loginfo(f"JG:{JG}")
            process_data(data_dict)
            # コールバック関数の処理が終わった後にdata_dictを初期化
            data_dict.clear()
            rospy.loginfo(f"---------------------------処理終了-----------------------------------\n")

        elif len(data_dict['pc1']) > 0: # pc1のデータのみ受信した場合
            #JG += 0
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
            #JG += 0
            rospy.loginfo(f"受信したpc2のデータ：{pc_data}")
            rospy.loginfo(f"---統合済みIDの捜索---　　id_mappings_pc2：{id_mappings_pc2}")
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
            #JG += 0
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
        JG = 0

# 文字列を各要素に合った型に変換して、リストに追加
def parse_data(data_str):
    data_list = data_str.split(',')
    timestamp_str = data_list[0]
    timestamp = datetime.datetime.strptime(timestamp_str, "%Y-%m-%d %H:%M:%S.%f")
    # IDと角度をint型、世界座標と距離をfloat型にして一つのリストに格納
    data = [int(data_list[i]) if i % 5 == 1 or i % 5 == 4 else float(data_list[i]) for i in range(1, len(data_list))]

    return timestamp, data


def distance(coord1, coord2):
    # 2つの座標の距離を計算
    return math.sqrt((coord1[0] - coord2[0])**2 + (coord1[1] - coord2[1])**2)

def process_data(data_dict):
    global integration_data, JG
    integration_data = "" # パブリッシュするデータをまとめる変数

    pc1_data = data_dict['pc1']
    pc2_data = data_dict['pc2']

    # 時刻が最も近い物同士でマッチング
    time_diff = 0.0 # 時間差を格納
    min_time_diff = float('inf') # 最小時間を格納する変数
    allowable_time = 0.3 # 許容時間差[s]
    matched_data_pc1 = None
    matched_data_pc2 = None

    remained_data_pc1 = None # IDを統合しなかった残りのデータを格納する
    del_num_list = [] # 今回マッチしたのデータのインデックスを保存するリスト(これを使用して、残りのデータremained_data_pc1の更新を行う)


    S = 0 # 時間差判定用
    # 蓄積データから時間差が最小な組み合わせを探す
    for time_pc1, coord_pc1 in pc1_data.items():
        for time_pc2, coord_pc2 in pc2_data.items():
            time_diff = abs((time_pc1 - time_pc2).total_seconds()) # 時間差[s]を計算
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
        pc1_before_id = None # ID統合を行う場合に、統合前のIDを保持する変数
        pc2_before_id = None
        rospy.loginfo(f"判定対象データの時間差：{min_time_diff}\n                            判定対象データ\n                            {matched_data_pc1} \n                            {matched_data_pc2}")

        
        # 許容距離以内にある場合、IDを同じものに変更
        if matched_data_pc1 is not None and matched_data_pc2 is not None:
            # 既に統合済みのIDであった場合、そのIDに変更
            rospy.loginfo(f"統合済みIDリスト id_mappings_pc1：{id_mappings_pc1}")
            for i in range(int(len(matched_data_pc1)/5)):# 検出した人数分ループ
                for id_mapping in id_mappings_pc1:
                    original_id, merged_id = id_mapping
                    if matched_data_pc1[i * 5] == original_id:
                        rospy.loginfo(f"pc1：ID {original_id} は既に統合されています。統合後のID {merged_id} に変更します。")
                        matched_data_pc1[i * 5] = merged_id

            rospy.loginfo(f"統合済みIDリスト id_mappings_pc2：{id_mappings_pc2}")
            for j in range(int(len(matched_data_pc2)/5)):# 検出した人数分ループ
                for id_mapping in id_mappings_pc2:
                    original_id, merged_id = id_mapping
                    if matched_data_pc2[j * 5] == original_id:
                        rospy.loginfo(f"pc2：ID {original_id} は既に統合されています。統合後のID {merged_id} に変更します。")
                        matched_data_pc2[j * 5] = merged_id
            
            # 同じIDはデータリストから除く
            pc1_double_ID_index = [] # 同一のIDの添字を保管するリスト
            pc2_double_ID_index = []
            for i in range(int(len(matched_data_pc1)/5)):# 検出した人数分ループ
                for j in range(int(len(matched_data_pc2)/5)):# 検出した人数分ループ
                    if matched_data_pc1[i * 5] == matched_data_pc2[j * 5]:
                        pc1_double_ID_index.append(i*5) # IDを持つ添字:0, 5, 10, 15
                        pc2_double_ID_index.append(j*5)
                        rospy.loginfo(f"同一IDを発見。pc1のID:{matched_data_pc1[i * 5]}, pc2のID:{matched_data_pc2[j * 5]}")
                        integration_data += compare_dist(matched_data_pc1[i*5 :i*5 + 5], matched_data_pc2[j*5 :j*5 + 5])
            # IDを昇順に並べる
            sorted_pc1_del_index_list = sorted(pc1_double_ID_index, reverse=False)
            sorted_pc2_del_index_list = sorted(pc2_double_ID_index, reverse=False)
            for i in range(int(len(sorted_pc1_del_index_list))):
                if len(matched_data_pc1[(sorted_pc1_del_index_list[i] - i*5) : (sorted_pc1_del_index_list[i] - i*5 + 5)]) == 5: # 2024.2.29追加。エラー回避用
                    rospy.loginfo(f"pc1 マッチングリストから削除：{matched_data_pc1[(sorted_pc1_del_index_list[i] - i*5)]}")
                    # xx
                    # 人間に近い方のカメラデータ(世界座標)を採用する
                    #integration_data += convert_to_string(matched_data_pc1[(sorted_pc1_del_index_list[i] - i*5) : (sorted_pc1_del_index_list[i] - i*5 + 4)]) # 距離を含まない人物情報をpub_dataに追加
                    del matched_data_pc1[(sorted_pc1_del_index_list[i] - i*5) : (sorted_pc1_del_index_list[i] - i*5 + 5)]
            for j in range(int(len(sorted_pc2_del_index_list))):
                rospy.loginfo(f"pc2 マッチングリストから削除：{matched_data_pc2[(sorted_pc2_del_index_list[j] - j*5)]}")
                del matched_data_pc2[(sorted_pc2_del_index_list[j] - j*5) : (sorted_pc2_del_index_list[j] - j*5 + 5)]
                # xx



            
            remained_data_pc1 = matched_data_pc1.copy()

            # 統合対象の探索
            for z in range(int(len(matched_data_pc1)/5)):# 検出した人数分ループ
                # rospy.loginfo(f"range : {range(int(len(matched_data_pc1)/4))}")
                change_num_pc1 = None # IDを変更するデータの添字(i)を格納
                change_num_pc2 = None # IDを変更するデータの添字(j)を格納
                min_distance = float('inf')  # 最短距離を格納する変数
                #rospy.loginfo(f"\nmatch_pc1: {matched_data_pc1}\nmatch_pc2: {matched_data_pc2}")
                for j in range(int(len(matched_data_pc2)/5)):# 検出した人数分ループ
                    # 両者(二人)の距離を求める(世界座標x-y平面上)
                    current_distance = distance(matched_data_pc1[z * 5 + 1: z * 5 + 2 + 1],
                                                matched_data_pc2[j * 5 + 1: j * 5 + 2 + 1])
                    current_distance = float(format(current_distance, '.3f')) # 小数第三位まで。floatに変換しないとエラーが起こる。
                    #rospy.loginfo(f"ループ{z}-{j}　　比較対象：{matched_data_pc1[z * 4]}と{matched_data_pc2[j * 4]}　　2点間の距離：{current_distance}[m]")
                    if current_distance <= allowable_dist and current_distance < min_distance:
                        min_distance = current_distance
                        change_num_pc1 = z * 5
                        change_num_pc2 = j * 5
                # zのループが一周するごとにIDを統合
                if change_num_pc1 is not None and change_num_pc2 is not None: 
                    if matched_data_pc1[change_num_pc1] != matched_data_pc2[change_num_pc2]:# マッチしたデータのIDが同一でない場合
                        #rospy.loginfo(f"\n\n\n\n\n")
                        rospy.loginfo("＜ID統合＞ \n                            Matching IDs: %d and %d", matched_data_pc1[change_num_pc1], matched_data_pc2[change_num_pc2])
                        rospy.loginfo(f"マッチしたペアの距離：{min_distance}[m]")

                        del_num_list.append(change_num_pc1)
                        #rospy.loginfo(f"del_num_list : {del_num_list}")
                        rospy.loginfo(f"辞書：{DetectionTime_dict}")
                        #detection_time_pc1 = DetectionTime_dict[str(matched_data_pc1[change_num_pc1])] # 検出時刻の比較に使用
                        detection_time_pc1 = DetectionTime_dict[matched_data_pc1[change_num_pc1]]
                        detection_time_pc2 = DetectionTime_dict[matched_data_pc2[change_num_pc2]]

                        if(detection_time_pc1 <= detection_time_pc2): # 先に検出された方のIDに統合
                            pc2_before_id = matched_data_pc2[change_num_pc2]
                            matched_data_pc2[change_num_pc2] = matched_data_pc1[change_num_pc1]
                            # 統合したIDのもとのIDと変更先IDをまとめるリストに追加
                            id_mappings_pc2.append((pc2_before_id, matched_data_pc2[change_num_pc2]))
                            # 距離データの比較
                            # カメラからの距離が近い方の世界座標を採用
                            #if matched_data_pc1[change_num_pc1+4] <= matched_data_pc2[change_num_pc2+4]:
                            integration_data += compare_dist(matched_data_pc1[change_num_pc1:change_num_pc1+5], matched_data_pc2[change_num_pc2:change_num_pc2+5])
                        else:
                            pc1_before_id = matched_data_pc1[change_num_pc1]
                            matched_data_pc1[change_num_pc1] = matched_data_pc2[change_num_pc2]
                            # 統合したIDのもとのIDと変更先IDをまとめるリストに追加
                            id_mappings_pc1.append((pc1_before_id, matched_data_pc1[change_num_pc1]))
                            integration_data += compare_dist(matched_data_pc1[change_num_pc1:change_num_pc1+5], matched_data_pc2[change_num_pc2:change_num_pc2+5])
                            #integration_data += convert_to_string(matched_data_pc1[change_num_pc1 : change_num_pc1 + 4])# cam1のデータを優先

                        rospy.loginfo("変更後の IDs: %d and %d", matched_data_pc1[change_num_pc1], matched_data_pc2[change_num_pc2])
                        rospy.loginfo(f"-----------------------------------------------------------")
                        judge = False

                        # マッチした要素(人間)をリストから除く。二重マッチ防止。
                        del matched_data_pc2[change_num_pc2:change_num_pc2+5]

            # IDを統合しなかった(それぞれのカメラでしか認識していない)人間のデータを追加
            #rospy.loginfo(f"del_num_list : {del_num_list}")
            if del_num_list != []:
                for i in range(int(len(del_num_list))):
                    del remained_data_pc1[(del_num_list[i] - i*5) : (del_num_list[i] - i*5 + 5)]
            # 残ったデータの追加
            for i in range(int(len(remained_data_pc1)/5)):
                integration_data += convert_to_string(remained_data_pc1[i*5:i*5+4])
            for j in range(int(len(matched_data_pc2)/5)):# 残った人数分ループ
                #rospy.loginfo(f"最終のmatch_pc2:{matched_data_pc2}")
                integration_data += convert_to_string(matched_data_pc2[j*5:j*5+4])
            
        if judge:
            rospy.loginfo(f"ID統合はなし")

        rospy.loginfo(f"pc1の統合済みIDリスト(元のIDと変更先ID): {id_mappings_pc1}")
        rospy.loginfo(f"pc2の統合済みIDリスト(元のIDと変更先ID): {id_mappings_pc2}")

        if integration_data != "":
            pub_integration_data.publish(integration_data[:-1]) # 文字列末尾の","を削除したものをパブリッシュ
            rospy.loginfo(f"パブリッシュ integration_data: {integration_data[:-1]}")

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
        rospy.loginfo(f"チェック：{result}")
        # もし角度がcam1で取得できていなかったら、cam2のデータを使用する
        if result[3] == -1:
            rospy.loginfo(f"1_角度データを補填：変更前は{result[3]}、変更後は{lst2[3]}")
            result[3] = lst2[3]
        result = convert_to_string(result[0:4])
    else:
        result = lst2[0:4]
        rospy.loginfo(f"チェック：{result}")
        # もし角度がcam2で取得できていなかったら、cam1のデータを使用する
        if result[3] == -1:
            rospy.loginfo(f"2_角度データを補填：変更前は{result[3]}、変更後は{lst1[3]}")
            result[3] = lst1[3]
        result = convert_to_string(result[0:4])

    rospy.loginfo(f"結果を出力します：{result[:-1]}")    
    return result


def listener():
    global pub_integration_data, JG
    rospy.init_node("listener", anonymous=True)

    # パブリッシャの作成
    pub_integration_data = rospy.Publisher("integration_data", String, queue_size=10)    

    # データを格納する辞書
    data_dict = defaultdict(dict)

    JG = 0 # パブリッシュ数調整用変数
    rospy.Subscriber("all_data_pc1", String, callback, callback_args=("pc1", data_dict))
    rospy.Subscriber("all_data_pc2", String, callback, callback_args=("pc2", data_dict))
    rospy.Subscriber("all_data_pc3", String, callback, callback_args=("pc3", data_dict)) # xxx
    #rospy.Subscriber("/iwasaki_2/all_data_pc2", String, callback, callback_args=("pc2", data_dict))

    rospy.spin()

if __name__ == "__main__":
    listener()


