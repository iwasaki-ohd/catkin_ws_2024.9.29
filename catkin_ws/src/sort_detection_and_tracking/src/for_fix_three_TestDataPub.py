#!/usr/bin/env python
# 2024.4.17
# テストデータをパブリッシュ
# 角度を含むデータ 
# 2024.4.26
# 距離も含む
# 2024.4.30
# カメラ3台分のデータ
# cam1-3で同じ位置に人間がいるケースはありえないことに注意

import rospy
from std_msgs.msg import String
import time
import random
from datetime import datetime

def publish_data():
    # ノードの初期化
    rospy.init_node('custom_publisher_node', anonymous=True)

    # パブリッシャーの作成
    pub1 = rospy.Publisher("all_data_pc1", String, queue_size=10)
    pub2 = rospy.Publisher("all_data_pc2", String, queue_size=10)
    pub3 = rospy.Publisher("all_data_pc3", String, queue_size=10)

    # ループでデータをパブリッシュ
    rate = rospy.Rate(2)  # ()Hzでループ
    A = 0 # パブリッシュ頻度調整用

    while not rospy.is_shutdown():
        # 現在時刻の取得
        #current_time = time.strftime("%Y-%m-%d %H:%M:%S")
        timestamp = rospy.get_time()
        current_time = datetime.fromtimestamp(timestamp) # 日付と時刻
        only_time = current_time.time()  # 時刻のみ
        current_time_1 = current_time
        current_time_2 = current_time
        current_time_3 = current_time

        # 特定の整数の生成（例: 0から100のランダムな整数）
        #random_integer = random.randint(0, 100)
        ID_1_1 = 1
        ID_1_2 = 2
        ID_1_3 = 100
        ID_2_1 = 21
        ID_2_2 = 22
        ID_2_3 = 23
        ID_2_4 = 200
        ID_3_1 = 31
        ID_3_2 = 32
        ID_3_3 = 300

        # 座標の生成（例: ランダムな座標）
        x_coordinate_1_1 = random.uniform(1.0, 1.1)
        y_coordinate_1_1 = random.uniform(1.0, 1.1)
        arc_1_1 = -1
        dist_1_1 = 0.5

        x_coordinate_1_2 = random.uniform(1.0, 1.1)
        y_coordinate_1_2 = random.uniform(1.0, 1.1)
        arc_1_2 = 12
        dist_1_2 = 0.8
        
        x_coordinate_1_3 = random.uniform(2.0, 2.1)
        y_coordinate_1_3 = random.uniform(2.0, 2.1)
        arc_1_3 = 13
        dist_1_3 = 1.0

        # 検証用
        x_coordinate_2_1 = random.uniform(1.0, 1.1)
        y_coordinate_2_1 = random.uniform(1.0, 1.1)
        arc_2_1 = 21
        dist_2_1 = 0.7

        x_coordinate_2_2 = random.uniform(1.0, 1.1)
        y_coordinate_2_2 = random.uniform(1.0, 1.1)
        arc_2_2 = -1
        dist_2_2 = 0.5

        x_coordinate_2_3 = random.uniform(5.0, 5.1)
        y_coordinate_2_3 = random.uniform(5.0, 5.1)
        arc_2_3 = -1
        dist_2_3 = 1.2

        x_coordinate_2_4 = random.uniform(3.0, 3.1)
        y_coordinate_2_4 = random.uniform(3.0, 3.1)
        arc_2_4 = 24
        dist_2_4 = 0.4

        # カメラ3台目
        x_coordinate_3_1 = random.uniform(5.0, 5.1)
        y_coordinate_3_1 = random.uniform(5.0, 5.1)
        arc_3_1 = 31
        dist_3_1 = 0.3

        x_coordinate_3_2 = random.uniform(6.0, 6.1)
        y_coordinate_3_2 = random.uniform(6.0, 6.1)
        arc_3_2 = -1
        dist_3_2 = 0.5

        x_coordinate_3_3 = random.uniform(3.0, 3.1)
        y_coordinate_3_3 = random.uniform(3.0, 3.1)
        arc_3_3 = -1
        dist_3_3 = 1.5


        # データの結合
        #data_string_1 = f"{current_time_1},{ID_1_1},{x_coordinate_1_1},{y_coordinate_1_1}" # 一人分
        #data_string_2 = f"{current_time_2},{ID_2_1},{x_coordinate_2_1},{y_coordinate_2_1}"
        #data_string_1 = f"{current_time_1},{ID_1_1},{x_coordinate_1_1},{y_coordinate_1_1},{ID_1_2},{x_coordinate_1_2},{y_coordinate_1_2},{ID_1_3},{x_coordinate_1_3},{y_coordinate_1_3}"
        #data_string_2 = f"{current_time_2},{ID_2_1},{x_coordinate_2_1},{y_coordinate_2_1},{ID_2_2},{x_coordinate_2_2},{y_coordinate_2_2},{ID_2_3},{x_coordinate_2_3},{y_coordinate_2_3},{ID_2_4},{x_coordinate_2_4},{y_coordinate_2_4}"

        #data_string_1 = f"{current_time_1},{ID_1_1},{x_coordinate_1_1},{y_coordinate_1_1},{ID_1_3},{x_coordinate_1_3},{y_coordinate_1_3}"
        #data_string_2 = f"{current_time_2},{ID_2_1},{x_coordinate_2_1},{y_coordinate_2_1},{ID_2_4},{x_coordinate_2_4},{y_coordinate_2_4}"

        # 角度あり
        #data_string_1 = f"{current_time_1},{ID_1_1},{x_coordinate_1_1},{y_coordinate_1_1},{arc_1_1}" # 一人分
        #data_string_2 = f"{current_time_2},{ID_2_1},{x_coordinate_2_1},{y_coordinate_2_1},{arc_2_1}"

        #data_string_1 = f"{current_time_1},{ID_1_1},{x_coordinate_1_1},{y_coordinate_1_1},{arc_1_1},{ID_1_2},{x_coordinate_1_2},{y_coordinate_1_2},{arc_1_2},{ID_1_3},{x_coordinate_1_3},{y_coordinate_1_3},{arc_1_3}"
        #data_string_2 = f"{current_time_2},{ID_2_1},{x_coordinate_2_1},{y_coordinate_2_1},{arc_2_1},{ID_2_2},{x_coordinate_2_2},{y_coordinate_2_2},{arc_2_2},{ID_2_3},{x_coordinate_2_3},{y_coordinate_2_3},{arc_2_3},{ID_2_4},{x_coordinate_2_4},{y_coordinate_2_4},{arc_2_4}"

        # 距離あり
        #data_string_1 = f"{current_time_1},{ID_1_1},{x_coordinate_1_1},{y_coordinate_1_1},{arc_1_1},{dist_1_1}" # 一人分
        #data_string_2 = f"{current_time_2},{ID_2_1},{x_coordinate_2_1},{y_coordinate_2_1},{arc_2_1},{dist_2_1}"
        #data_string_3 = f"{current_time_3},{ID_3_1},{x_coordinate_3_1},{y_coordinate_3_1},{arc_3_1},{dist_3_1}"

        data_string_1 = f"{current_time_1},{ID_1_1},{x_coordinate_1_1},{y_coordinate_1_1},{arc_1_1},{dist_1_1},{ID_1_2},{x_coordinate_1_2},{y_coordinate_1_2},{arc_1_2},{dist_1_2},{ID_1_3},{x_coordinate_1_3},{y_coordinate_1_3},{arc_1_3},{dist_1_3}"
        data_string_2 = f"{current_time_2},{ID_2_1},{x_coordinate_2_1},{y_coordinate_2_1},{arc_2_1},{dist_2_1},{ID_2_2},{x_coordinate_2_2},{y_coordinate_2_2},{arc_2_2},{dist_2_2},{ID_2_3},{x_coordinate_2_3},{y_coordinate_2_3},{arc_2_3},{dist_2_3},{ID_2_4},{x_coordinate_2_4},{y_coordinate_2_4},{arc_2_4},{dist_2_4}"
        data_string_3 = f"{current_time_3},{ID_3_1},{x_coordinate_3_1},{y_coordinate_3_1},{arc_3_1},{dist_3_1},{ID_3_2},{x_coordinate_3_2},{y_coordinate_3_2},{arc_3_2},{dist_3_2}"
        

        """
        # パブリッシュ通常版
        pub1.publish(data_string_1)
        # データをパブリッシュ
        rospy.loginfo(f"pub1：{data_string_1}")
        A +=1
        if A % 5 == 0:
            pub2.publish(data_string_2)
            rospy.loginfo(f"pub2：{data_string_2}")
        """
        # パブリッシュ限定版
        if A % 5 == 0:
            pub1.publish(data_string_1)
            pub2.publish(data_string_2)
            pub3.publish(data_string_3)
            rospy.loginfo(f"\npub1：{data_string_1}\npub2：{data_string_2}\npub3：{data_string_3}")
            #rospy.loginfo(f"\npub1：{data_string_1}\npub2：{data_string_2}")
            pass
        elif A % 5 == 1:
            pub1.publish(data_string_1)
            rospy.loginfo(f"pub1：{data_string_1}")
            #rospy.loginfo(f"なし")
            pass
        elif A % 5 == 2:
            pub2.publish(data_string_2)
            rospy.loginfo(f"pub2：{data_string_2}")
            pass
        elif A % 5 == 3:
            pub3.publish(data_string_3)
            rospy.loginfo(f"pub3：{data_string_3}")
            pass
        elif A % 5 == 4:
            pub3.publish(data_string_3)
            rospy.loginfo(f"pub3_2：{data_string_3}")
            pass

        A +=1
        rospy.loginfo(f"--------------------------------------------------")
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_data()
    except rospy.ROSInterruptException:
        pass
