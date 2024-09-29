#!/usr/bin/env python
# coding: UTF-8

# add_rule~~の判定ラインないバージョン
# 判定ラインによる経路変更機能も削除
# A*_path_plannningの変更バージョン
# もととなるプログラム(8.20)

import rospy
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import cv2
import math


t = 2   # 何秒後を予測するかを決定する変数
mode = 0 # シチュエーション切り替え用変数。mode1がルート変更モード

#　ここでロボットの移動について定義している。
times = 60.0 # データのサブスクライブ回数
x_distance=1.2 # 目的地までの移動距離
step = 3 # stepはロボットがルート移動を終えるまでに必要な時間(times)

cv = 80 # スケーリング用変数(cv2の表示のために使用)

# cv領域のサイズ
width = 600
height = 600

# 四角形の位置を記録するリストを初期化
positions = []


class KalmanFilterNode:
    def __init__(self):
        rospy.init_node('kalman_filter_node', anonymous=True)

        # ROS Parameters
        self.prev_time = rospy.get_time()  
        self.dt = 0.2 
        self.R_std = 1  
        self.theta_p = -1  
        self.judge = 0  
        self.danger = 0  
        self.timer = 0  
        self.check = 0 
        self.moving_count = 0
        self.y_moving=0
        self.y_count=0
        self.avoidance_threshold = 1.5  # 回避する距離の閾値
        self.human_position = None  # 人間の現在位置を保存
        self.human_pre_position = None  # 人間の予測位置を保存

        # カルマンフィルターの初期化
        self.init_filter()

        # ROS Subscribers
        rospy.Subscriber("/man_made_data", String, self.sensor_callback)

        # ROS Publishers
        self.predicted_pub = rospy.Publisher('/kalman_filter/predicted_position', PoseStamped, queue_size=10)

        # 観測値の軌跡と予測値の軌跡の初期化
        self.observation_trajectory = []
        self.prediction_trajectory = []

        # 現在のロボットの位置(画像座標)
        self.robot_position = [200, 320]
        self.goal_position=(520, 320)  # ゴールの設置
        #rospy.loginfo(f"self.robit_position{self.robot_position[0]}, {self.robot_position[1]}")


    def init_filter(self):
        # カルマンフィルターの初期化
        #self.x = np.zeros((6, 1))  # 初期状態
        self.x = np.array([[1.55], [2.9], [0.], [0.], [0.], [0.]]) # JSST用初期値
        self.P = np.eye(6) * 100.0  # 共分散行列
        self.F = np.array([[1., 0., 0., self.dt, 0., 0.],
                           [0., 1., 0., 0., self.dt, 0.],
                           [0., 0., 1., 0., 0., self.dt],
                           [0., 0., 0., 1., 0., 0.],
                           [0., 0., 0., 0., 1., 0.],
                           [0., 0., 0., 0., 0., 1.]])  # 状態遷移行列
        self.H = np.array([[1., 0., 0., 0., 0., 0.],
                           [0., 1., 0., 0., 0., 0.],
                           [0., 0., 1., 0., 0., 0.]])  # 観測行列
        #self.Q = np.diag([1, 1, 1, 10.0, 10.0, 10.0])
        self.Q = np.diag([0.5, 0.5, 0.5, 5.0, 5.0, 5.0])  # システムノイズの共分散行列
        #self.Q = np.diag([0.49, 0.49, 0.49, 5.175, 5.175, 5.175])  # システムノイズの共分散行列
        self.R = np.diag([self.R_std ** 2, self.R_std ** 2, self.R_std ** 2])  # 観測ノイズの共分散行列
        #self.R = np.diag([0, 0, 0])

    def predict(self):
        # 予測ステップ
        self.x = np.dot(self.F, self.x)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q
        self.x[2, 0] = self.x[2, 0] % 360  # 角度を0〜360に正規化 xx

    def update(self, measurement):
        # フィルタリングステップ
        angle_diff = measurement[2] - self.x[2, 0]
        angle_diff = (angle_diff + 180) % 360 - 180  # 最短回転方向を考慮 xx

        #y = np.array(measurement).reshape(-1, 1) - np.dot(self.H, self.x)
        y = np.array([measurement[0], measurement[1], self.x[2, 0] + angle_diff]).reshape(-1, 1) - np.dot(self.H, self.x)
        S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        self.x = self.x + np.dot(K, y)
        self.P = np.dot((np.eye(6) - np.dot(K, self.H)), self.P)
        self.x[2, 0] = self.x[2, 0] % 360  # 角度を0〜360に正規化 xx

    def sensor_callback(self, data):
        # センサーデータを受信したときの処理
        current_time = rospy.get_time()
        self.dt = current_time - self.prev_time
        self.prev_time = current_time

        data_fixed = ','.join(data.data.split(',')).replace(',,', ',')
        data_str = data_fixed.split(',')

        # データの解析と処理
        try:
            #rospy.loginfo(f"確認：{self.theta_p}")
            time_str = data_str[0]
            id_val = int(data_str[1])
            x = float(data_str[2])
            y = float(data_str[3])
            theta = float(data_str[4]) % 360  # 角度を0〜360に正規化

            if float(data_str[4]) < 0 and self.theta_p >= 0: # 取得角度が-1の時、角度が取得できなかったことを表す。このときは、前回の予測角度を現在の取得角度として扱う。
                theta = self.theta_p
                #rospy.loginfo(f"角度が取得できなかったため前回の予測角度を再利用します。正規化{(theta)},θ={(data_str[4])}")

            # フィルターを更新
            self.update([x, y, theta])

            # 人間の位置を更新(8.20)
            self.human_position = (x * cv + width//2, -y * cv + height//2)
            self.human_pre_position = (int((self.x[0, 0] + self.x[3, 0] * t) * cv)+ width//2, int((self.x[1, 0] + self.x[4, 0] * t) * cv) * (-1)+ height//2)
            # ロボットが人間から十分に離れているか確認(8.20)
            self.check_and_avoid_collision()
            

            # 予測された状態をパブリッシュ
            predicted_msg = PoseStamped()
            predicted_msg.header.stamp = rospy.Time.now()
            predicted_msg.pose.position.x = self.x[0, 0]
            predicted_msg.pose.position.y = self.x[1, 0]
            predicted_msg.pose.orientation.z = self.x[2, 0]
            self.predicted_pub.publish(predicted_msg)

            future_x = self.x[0, 0] + self.x[3, 0] * t
            future_y = self.x[1, 0] + self.x[4, 0] * t
            future_theta = self.x[2, 0] + self.x[5, 0] * t
            #rospy.loginfo(f"{t}秒後のPredicted Position: x={future_x}, y={future_y}, θ={future_theta}")

            future_direction = math.atan2(self.x[3, 0], self.x[4, 0]) # 速度方向を計算
            future_direction_deg = (math.degrees(future_direction)+270)%360 # 右をx軸正、下をy軸正とする
            #rospy.loginfo(f"進行方向：{future_direction_deg}[°]")

            rospy.loginfo(f"--------------------------------------------------------------")
            # 観測値の軌跡をストック
            o_posi = [int(x * cv), int(y * cv) * (-1)]
            self.observation_trajectory.append((o_posi[0]+width//2, o_posi[1]+height//2)) # width//2を足すことで、cv2の中央を世界座標(x, y)=(0, 0)としている

            # 予測値の軌跡をストック
            #p_posi = [int((self.x[0, 0] + self.x[3, 0] * self.dt) * cv), int((self.x[1, 0] + self.x[4, 0] * self.dt) * cv) * (-1)]
            p_posi = [int((self.x[0, 0] + self.x[3, 0] * t) * cv), int((self.x[1, 0] + self.x[4, 0] * t) * cv) * (-1)]
            self.prediction_trajectory.append((p_posi[0]+width//2, p_posi[1]+height//2))

            # OpenCVを使用して可視化
            self.visualize_position()


            self.theta_p = self.x[2, 0] # 取得角度が-1の時に使われる

        except IndexError:
            rospy.logerr("Invalid sensor data format: {}".format(data.data))
        except ValueError:
            rospy.logerr("Invalid sensor data values: {}".format(data.data))

    def check_and_avoid_collision(self): # (8.20)
        rospy.loginfo("check")
        if self.human_pre_position is None:
            rospy.loginfo("miss")
            return
        if len(self.prediction_trajectory) == 0:
            rospy.logwarn("No prediction trajectory available")
            return

        # ロボットの現在位置を計算
        robot_x = self.observation_trajectory[-1][0]
        robot_y = self.observation_trajectory[-1][1]

        # 人間との距離を計算
        distance =  math.sqrt(((self.robot_position[0]-self.prediction_trajectory[-1][0])/cv)**2 + ((self.robot_position[1]-self.prediction_trajectory[-1][1])/cv)**2)

        if distance < 1.0:
            rospy.loginfo("Avoiding human! Distance: {}".format(distance))
            self.avoid_human()
        #rospy.loginfo("OK")

    def avoid_human(self): # (8.20)
        # 避けるルートを生成
        self.y_moving += 50  # 避ける動きのシミュレーション（Y方向に移動）
        rospy.loginfo("Avoidance maneuver activated!")

    def draw_dotted_line(self, image, start_point, end_point, color, thickness, segment_length):
        """
        任意の二点間に点線を描画する関数。
    
        :param image: 描画対象の画像
        :param start_point: 点線の開始点 (x, y)
        :param end_point: 点線の終了点 (x, y)
        :param color: 点線の色 (B, G, R)
        :param thickness: 点線の太さ
        :param segment_length: 点線の各セグメントの長さ
        """
        line_vector = np.array(end_point) - np.array(start_point)
        line_length = np.linalg.norm(line_vector)
        line_direction = line_vector / line_length

        current_point = np.array(start_point)
        while np.linalg.norm(current_point - np.array(start_point)) < line_length:
            next_point = current_point + line_direction * segment_length
            if np.linalg.norm(next_point - np.array(start_point)) > line_length:
                next_point = np.array(end_point)
            cv2.line(image, tuple(current_point.astype(int)), tuple(next_point.astype(int)), color, thickness)
            current_point = next_point + line_direction * segment_length

    def visualize_position(self):
        self.timer +=1
        #rospy.loginfo(f"timer {self.timer}")

        # カレントポジションと次の予測位置を可視化
        image = np.zeros((width, height, 3), dtype=np.uint8)  # 仮の画像を作成
        
        # 観測値の軌跡を描画
        if hasattr(self, 'observation_trajectory'):
            for i in range(1, len(self.observation_trajectory)):
                cv2.line(image, self.observation_trajectory[i - 1], self.observation_trajectory[i], (50, 255, 50), 1)
               # 通過点に小さな円を描画
                cv2.circle(image, self.observation_trajectory[i], 2, (50, 255, 50), -1)

        # 予測値の軌跡を描画
        if hasattr(self, 'prediction_trajectory'):
            for i in range(1, len(self.prediction_trajectory)):
                cv2.line(image, self.prediction_trajectory[i - 1], self.prediction_trajectory[i], (50, 50, 255), 1)
                # 通過点に小さな円を描画
                cv2.circle(image, self.prediction_trajectory[i], 2, (50, 50, 255), -1)

        # カレントポジションを描画  
        cv2.circle(image, (self.observation_trajectory[-1][0], self.observation_trajectory[-1][1]), 5, (0, 255, 0), -1)
        # 次の予測位置を描画
        cv2.circle(image, (self.prediction_trajectory[-1][0], self.prediction_trajectory[-1][1]), 5, (0, 0, 255), -1)


        # 人間の位置を描画(8.20)  重複してる？
        """
        if self.human_pre_position:
            cv2.circle(image, self.human_pre_position, 5, (0, 255, 255), -1)
            """

        # 研究室のマップを表示
        map_coordinates = [((-1.5,-2), (3.2, -2)), # 通路拡大(一番下の線)
               ((-1.5,0), (0.1, 0)),
               ((0.1, 0), (0.1, 2.4)),
               ((0.1,2.4), (-1.5, 2.4)),
               ((-1.5,3.2), (3.2, 3.2)),
               ((3.2,3.2), (3.2, -0.0)), # 通路拡大
               ((3.2,0), (2, 0)),
               ((2,0), (2, 2.4)),
               ((2,2.4), (3.2, 2.4))
               ]

        for coord_pair in map_coordinates:
            pt1, pt2 = coord_pair

            # 座標を整数に変換してから線を引く
            pt1 = (int(pt1[0]*cv+width/2), int(-1*pt1[1]*cv+height/2))  # 座標のスケーリング
            pt2 = (int(pt2[0]*cv+width/2), int(-1*pt2[1]*cv+height/2))  # 座標のスケーリング
            cv2.line(image, pt1, pt2, (255, 255,255), 2)


        
        # 各ルートの境目
        start_point_AB = (int(-1.5*cv +width/2), int(-1*(-0.5)*cv+height/2))


        # ゴールの設置
        cv2.circle(image, self.goal_position, 8, (255, 0, 255), -1)


        # ロボットの描画・軌跡の表示

        # ロボットの位置を計算
        self.robot_position[0] = start_point_AB[0] + int(self.timer * x_distance * cv / times)
        self.robot_position[1] = start_point_AB[1] - 15 + self.y_moving - 8
        """
        if self.y_moving !=0:
            self.y_moving = 0
            """
        
        # 現在の位置をリストに追加
        positions.append((self.robot_position[0], self.robot_position[1]))

        for i in range(1, len(positions)):
            cv2.line(image, positions[i-1], positions[i], (255, 255, 200), 1, cv2.LINE_AA)
            cv2.circle(image, positions[i], 5, (255, 0, 0), -1)
        # 軌道生成・表示----------------------------------------------------------------------------

        cv2.line(image, self.robot_position, self.goal_position, (0, 165, 255), 2)


        # 説明の追加
        cv2.rectangle(image, (10, 150), (210, 210), (50, 50, 50), -1) # 背景
        # テキストを描画
        cv2.circle(image, (25, 160), 5, (0, 0, 255), -1)
        cv2.putText(image, ":Observed position", (40, 165), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.circle(image, (25, 180), 5, (0, 255, 0), -1)
        cv2.putText(image, ":Predicted position", (40, 185), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.circle(image, (25, 200), 5, (255, 0, 255), -1)
        cv2.putText(image, ":Robot's destination", (40, 205), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        
        # ロボットと人間の距離を計算
        distance_rp = math.sqrt(((self.robot_position[0]-self.prediction_trajectory[-1][0])/cv)**2 + ((self.robot_position[1]-self.prediction_trajectory[-1][1])/cv)**2)
        # ゴールまでの距離
        distance_rg =  math.sqrt(((self.robot_position[0]-self.goal_position[0])/cv)**2 + ((self.robot_position[1]-self.goal_position[1])/cv)**2)

        # 距離を表示
        cv2.rectangle(image, (10, 230), (295, 270), (50, 50, 50), -1) # 背景
        formatted_distance_rp = "{:.3f}".format(distance_rp)  # 小数点以下3桁までフォーマット
        Text_rp = f"Distance to person(pre):{formatted_distance_rp}[m]"
        formatted_distance_rg = "{:.3f}".format(distance_rg)  # 小数点以下3桁までフォーマット
        Text_rg = f"Distance to destination :{formatted_distance_rg}[m]"
        cv2.putText(image, Text_rp, (20, 245), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(image, Text_rg, (20, 265), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # 画像を表示
        cv2.imshow("Position Visualization", image)
        cv2.waitKey(1)
        
    def run(self):
        rate = rospy.Rate(1.0 / self.dt)
        while not rospy.is_shutdown():
            # 予測ステップを実行
            self.predict()
            rate.sleep()

if __name__ == "__main__":
    try:
        kf_node = KalmanFilterNode()
        kf_node.run()
    except rospy.ROSInterruptException:
        pass