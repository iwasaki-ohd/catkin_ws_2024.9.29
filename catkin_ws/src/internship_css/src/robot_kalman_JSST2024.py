#!/usr/bin/env python
# coding: UTF-8

# /all_data_pc1に含まれているデータ↓
# "時刻、人物1のデータ(ID, 世界座標X, 世界座標Y, 角度[°], カメラとの距離[m])"

# 判定ラインによる出力変更条件式を追加(5.14)
# ルートやロボットの追加(ロボットの動きはなめらかでない)_7.15

import rospy
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import cv2
import math

t = 2 # 何秒後を予測するかを決定する変数
cv = 80 # スケーリング用変数(cv2の表示のために使用)
mode = 1 # シチュエーション切り替え用変数。mode1がルート変更モード

# cv領域のサイズ
width = 600
height = 600
# カレントポジションと次の予測位置を可視化
#image = np.zeros((width, height, 3), dtype=np.uint8)  # 仮の画像を作成

class KalmanFilterNode:
    def __init__(self):
        rospy.init_node('kalman_filter_node', anonymous=True)

        # ROS Parameters
        self.prev_time = rospy.get_time()  # 前回の時間
        self.dt = 0.3  # 時間差、初期値
        self.R_std = 1.0  # 観測ノイズの標準偏差

        self.theta_p = -1 # 角度が取得できなかった時用
        self.judge = 0 # 判定ライン用
        self.danger = 0 # ロボットがいる方向に曲がってくる可能性(三段階評価) ❍❍❍   
    

        # カルマンフィルターの初期化
        self.init_filter()

        # ROS Subscribers
        rospy.Subscriber("/all_data_pc1", String, self.sensor_callback)

        # ROS Publishers
        self.predicted_pub = rospy.Publisher('/kalman_filter/predicted_position', PoseStamped, queue_size=10)

        # 観測値の軌跡と予測値の軌跡の初期化
        self.observation_trajectory = []
        self.prediction_trajectory = []

    def init_filter(self):
        # カルマンフィルターの初期化
        #self.x = np.zeros((6, 1))  # 初期状態
        self.x = np.array([[1.5], [2.], [0.], [0.], [0.], [0.]]) # JSST用初期値
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
        self.Q = np.diag([0.5, 0.5, 0.5, 5.0, 5.0, 5.0])  # システムノイズの共分散行列
        #self.Q = np.diag([0.49, 0.49, 0.49, 5.175, 5.175, 5.175])  # システムノイズの共分散行列
        self.R = np.diag([self.R_std ** 2, self.R_std ** 2, self.R_std ** 2])  # 観測ノイズの共分散行列

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
            #theta = float(data_str[4])
            theta = float(data_str[4]) % 360  # 角度を0〜360に正規化
            #rospy.loginfo(f"theta{theta}")

            if float(data_str[4]) < 0 and self.theta_p >= 0: # 取得角度が-1の時、角度が取得できなかったことを表す。このときは、前回の予測角度を現在の取得角度として扱う。
                theta = self.theta_p
                #rospy.loginfo(f"角度が取得できなかったため前回の予測角度を再利用します。正規化{(theta)},θ={(data_str[4])}")


            # フィルターを更新
            self.update([x, y, theta])
            
            """
            if self.x[2, 0] < 0: # 角度調整。0~360におさめる。
                self.x[2, 0] +=360
                rospy.loginfo("確認")
            """

            # 予測された状態をパブリッシュ
            predicted_msg = PoseStamped()
            predicted_msg.header.stamp = rospy.Time.now()
            predicted_msg.pose.position.x = self.x[0, 0]
            predicted_msg.pose.position.y = self.x[1, 0]
            predicted_msg.pose.orientation.z = self.x[2, 0]
            self.predicted_pub.publish(predicted_msg)

            #rospy.loginfo(f"時刻：{time_str}")
             # 現在地と予測移動先をターミナル上で出力
            #rospy.loginfo("Current Position: x={}, y={}, θ={}".format(self.x[0, 0], self.x[1, 0], self.x[2, 0]))
            #rospy.loginfo("Current Position: x={}, y={}, θ={}".format(x, x, theta))
            #rospy.loginfo(f"Current predicted Position: x={self.x[0, 0]}, y={self.x[1, 0]}, θ={self.x[2, 0]}")
            # t秒後の予測位置を表示
            # rospy.loginfo("Predicted Next Position: x={}, y={}".format(self.x[0, 0] + self.x[3, 0] * self.dt, self.x[1, 0] + self.x[4, 0] * self.dt))
            future_x = self.x[0, 0] + self.x[3, 0] * t
            future_y = self.x[1, 0] + self.x[4, 0] * t
            future_theta = self.x[2, 0] + self.x[5, 0] * t
            #rospy.loginfo(f"{t}秒後のPredicted Position: x={future_x}, y={future_y}, θ={future_theta}")

            future_direction = math.atan2(self.x[3, 0], self.x[4, 0]) # 速度方向を計算
            future_direction_deg = (math.degrees(future_direction)+270)%360 # 右をx軸正、下をy軸正とする
            #rospy.loginfo(f"進行方向：{future_direction_deg}[°]")
            # 判定ライン通過時に、右に曲がりそうかどうかを判定する条件分
            # 進行方向は固定であるとする(切り返しは考慮しない)
            # cv上では右がx軸正方向、下がy軸正方向
            if (1.0 <= future_y < 1.7 and self.judge ==0):
                self.judge += 1
                rospy.loginfo(f"*****Passed through judgment line 1*****")
                rospy.loginfo(f"Time：{time_str}")
                future_direction = math.atan2(self.x[3, 0], self.x[4, 0]) # 速度方向を計算
                future_direction_deg = (math.degrees(future_direction)+270)%360 # 右をx軸正、下をy軸正とする
                rospy.loginfo(f"Direction of travel：{future_direction_deg}[°]")
                if  0<= future_direction_deg <= 90:
                    rospy.loginfo("Driving Route: A →  A")
                elif 90 < future_direction_deg <= 180:
                    self.danger +=1
                    rospy.loginfo("Driving Route: A →  B")
                else:
                    rospy.loginfo("進行方向が反対です")
                    self.judge -=1

            if (0.3 <= future_y < 1.0 and self.judge ==1):
                self.judge += 1
                rospy.loginfo(f"*****Passed through judgment line 2*****")
                rospy.loginfo(f"Time：{time_str}")
                future_direction = math.atan2(self.x[3, 0], self.x[4, 0]) # 速度方向を計算
                future_direction_deg = (math.degrees(future_direction)+270)%360 # 右をx軸正、下をy軸正とする
                rospy.loginfo(f"Direction of travel：{future_direction_deg}[°]")
                if 0 <= future_direction_deg <= 90:
                    if self.danger == 1:
                        rospy.loginfo("Driving Route: B →  B")
                    else:
                        rospy.loginfo("Driving Route: A →  A")
                elif 90 < future_direction_deg <= 180:
                    self.danger +=1
                    if self.danger == 2:
                        rospy.loginfo("Driving Route: B →  C")
                    else:
                        rospy.loginfo("Driving Route: A →  C")
            if (future_y < 0.3 and self.judge ==2):
                self.judge += 1
                rospy.loginfo(f"*****Passed through judgment line 3*****")
                rospy.loginfo(f"Time：{time_str}")
                future_direction = math.atan2(self.x[3, 0], self.x[4, 0]) # 速度方向を計算
                future_direction_deg = (math.degrees(future_direction)+270)%360 # 右をx軸正、下をy軸正とする
                rospy.loginfo(f"Direction of travel：{future_direction_deg}[°]")
                if 0 <= future_direction_deg <= 90:
                    if self.danger == 2:
                        rospy.loginfo("Driving Route: C →  C")
                    elif self.danger == 1:
                        rospy.loginfo("Driving Route: B →  B")
                    else:
                        rospy.loginfo("Driving Route: A →  A")
                elif 90 < future_direction_deg <= 180:
                    self.danger +=1
                    if self.danger == 3:
                        rospy.loginfo("Driving Route: C →  D")
                    elif self.danger == 2:
                        rospy.loginfo("Driving Route: B →  D")
                    else:
                        rospy.loginfo("Driving Route: A →  D")


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

        # 判定ライン
        cv2.line(image, (int(0.1*cv +width/2), int(-1*0.3*cv+height/2)), (int(2*cv +width/2), int(-1*0.3*cv +height/2)), (100, 100,255), 2) # 3
        cv2.line(image, (int(0.1*cv +width/2), int(-1*1.0*cv+height/2)), (int(2*cv +width/2), int(-1*1.0*cv +height/2)), (100, 255,255), 2) # 2
        cv2.line(image, (int(0.1*cv +width/2), int(-1*1.7*cv+height/2)), (int(2*cv +width/2), int(-1*1.7*cv +height/2)), (255, 255,100), 2) # 1
        # 各ルートの境目
        """"
        cv2.line(image, (int(-1.5*cv +width/2), int(-1*(-0.5)*cv+height/2)), (int(3.2*cv +width/2), int(-1*(-0.5)*cv +height/2)), (100, 100,100), 2) # A-B
        cv2.line(image, (int(-1.5*cv +width/2), int(-1*(-1)*cv+height/2)), (int(3.2*cv +width/2), int(-1*(-1)*cv +height/2)), (100, 100,100), 2) # B-C
        cv2.line(image, (int(-1.5*cv +width/2), int(-1*(-1.5)*cv+height/2)), (int(3.2*cv +width/2), int(-1*(-1.5)*cv +height/2)), (100, 100,100), 2) # C-D
        """
        start_point_AB = (int(-1.5*cv +width/2), int(-1*(-0.5)*cv+height/2))
        end_point_AB = (int(3.2*cv +width/2), int(-1*(-0.5)*cv +height/2))
        start_point_BC = (int(-1.5*cv +width/2), int(-1*(-1)*cv+height/2))
        end_point_BC = (int(3.2*cv +width/2), int(-1*(-1)*cv +height/2))
        start_point_CD = (int(-1.5*cv +width/2), int(-1*(-1.5)*cv+height/2))
        end_point_CD = (int(3.2*cv +width/2), int(-1*(-1.5)*cv +height/2))
        color = (100, 100, 100)
        thickness = 2
        segment_length = 10 # 点線のセグメントの長さ

        self.draw_dotted_line(image, start_point_AB, end_point_AB, color, thickness, segment_length)
        self.draw_dotted_line(image, start_point_BC, end_point_BC, color, thickness, segment_length)
        self.draw_dotted_line(image, start_point_CD, end_point_CD, color, thickness, segment_length)

        # ロボットの描画
        """"
        time_to_goal = 14 # 移動時間
        move_y = self.danger *cv * 0.5
        rospy.loginfo(f"move_y{move_y}")
        """
        """"
        if(mode==1):
            cv2.rectangle(image, (start_point_AB[0], start_point_AB[1]-15+self.danger*40), (start_point_AB[0]+30, start_point_AB[1]-15-15+self.danger*40), (200, 100, 0), cv2.FILLED, cv2.LINE_AA)
            cv2.circle(image, (start_point_AB[0]+7, start_point_AB[1]-10+self.danger*40), 4, (200, 200, 200), -1)
            cv2.circle(image, (start_point_AB[0]+23, start_point_AB[1]-10+self.danger*40), 4, (200, 200, 200), -1)
        """
        if(mode==1):
            cv2.rectangle(image, (start_point_AB[0]+self.danger*32, start_point_AB[1]-15+self.danger*40), (start_point_AB[0]+30+self.danger*32, start_point_AB[1]-15-15+self.danger*40), (200, 100, 0), cv2.FILLED, cv2.LINE_AA)
            cv2.circle(image, (start_point_AB[0]+7+self.danger*32, start_point_AB[1]-10+self.danger*40), 4, (200, 200, 200), -1)
            cv2.circle(image, (start_point_AB[0]+23+self.danger*32, start_point_AB[1]-10+self.danger*40), 4, (200, 200, 200), -1)
        else:
            cv2.rectangle(image, (start_point_AB[0]+self.danger*40, start_point_AB[1]-15), (start_point_AB[0]+30+self.danger*40, start_point_AB[1]-15-15), (200, 100, 0), cv2.FILLED, cv2.LINE_AA)
            cv2.circle(image, (start_point_AB[0]+7+self.danger*40, start_point_AB[1]-10), 4, (200, 200, 200), -1)
            cv2.circle(image, (start_point_AB[0]+23+self.danger*40, start_point_AB[1]-10), 4, (200, 200, 200), -1)
        

        # 画像を表示
        cv2.imshow("Position Visualization", image)
        cv2.waitKey(1)
        

    def run(self):
        rate = rospy.Rate(1.0 / self.dt)
        while not rospy.is_shutdown():
            # 予測ステップを実行
            self.predict()

            """
            start_point_AB = (int(-1.5*cv +width/2), int(-1*(-0.5)*cv+height/2))
            if(mode==1):
                cv2.rectangle(image, (start_point_AB[0]+self.danger*32, start_point_AB[1]-15+self.danger*40), (start_point_AB[0]+30+self.danger*32, start_point_AB[1]-15-15+self.danger*40), (200, 100, 0), cv2.FILLED, cv2.LINE_AA)
                cv2.circle(image, (start_point_AB[0]+7+self.danger*32, start_point_AB[1]-10+self.danger*40), 4, (200, 200, 200), -1)
                cv2.circle(image, (start_point_AB[0]+23+self.danger*32, start_point_AB[1]-10+self.danger*40), 4, (200, 200, 200), -1)
            else:
                cv2.rectangle(image, (start_point_AB[0]+self.danger*40, start_point_AB[1]-15), (start_point_AB[0]+30+self.danger*40, start_point_AB[1]-15-15), (200, 100, 0), cv2.FILLED, cv2.LINE_AA)
                cv2.circle(image, (start_point_AB[0]+7+self.danger*40, start_point_AB[1]-10), 4, (200, 200, 200), -1)
                cv2.circle(image, (start_point_AB[0]+23+self.danger*40, start_point_AB[1]-10), 4, (200, 200, 200), -1)
            
            # 画像を表示
            cv2.imshow("Position Visualization", image)
            cv2.waitKey(1)
            """

            rate.sleep()

if __name__ == "__main__":
    try:
        kf_node = KalmanFilterNode()
        kf_node.run()
    except rospy.ROSInterruptException:
        pass
