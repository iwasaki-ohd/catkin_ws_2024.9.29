#!/usr/bin/env python
# coding: UTF-8

# add_rule~~の判定ラインないバージョン


import rospy
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import cv2
import math
import heapq
import random



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

# A*アルゴリズムの実装
class Node:
    def __init__(self, position, parent=None, g=0, h=0):
        self.position = position
        self.parent = parent
        self.g = g  # 開始からノードまでのコスト
        self.h = h  # ヒューリスティック（ノードからゴールまでの推定コスト）
        self.f = g + h  # 総コスト

    def __lt__(self, other):
        return self.f < other.f

def heuristic(a, b):
    return np.linalg.norm(np.array(a) - np.array(b))

def astar(grid, start, goal, human_position, human_radius):
    open_list = []
    closed_list = set()
    heapq.heappush(open_list, Node(start, None, 0, heuristic(start, goal)))

    while open_list:
        current_node = heapq.heappop(open_list)
        closed_list.add(current_node.position)

        if current_node.position == goal:
            path = []
            while current_node:
                path.append(current_node.position)
                current_node = current_node.parent
            return path[::-1]  # 経路を逆にして返す

        for direction in [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (-1, -1), (1, -1), (-1, 1)]:
            neighbor_position = (current_node.position[0] + direction[0], current_node.position[1] + direction[1])

            if 0 <= neighbor_position[0] < grid.shape[0] and 0 <= neighbor_position[1] < grid.shape[1]:
                if grid[int(neighbor_position[0]), int(neighbor_position[1])] == 1:
                    continue  # 障害物

                distance_to_human = np.linalg.norm(np.array(neighbor_position) - np.array(human_position))
                if distance_to_human < human_radius:
                    continue  # 危険エリアを回避

                neighbor_node = Node(neighbor_position, current_node, current_node.g + 1, heuristic(neighbor_position, goal))

                if neighbor_position in closed_list:
                    continue

                heapq.heappush(open_list, neighbor_node)

    return None  # 経路が見つからなかった

# マップ設定
grid_size = 100
grid = np.zeros((grid_size, grid_size))

# 開始地点、ゴール地点、及び人の位置
start = (int(-1.5*cv +width/2), int(-1*(-0.5)*cv+height/2)) # start_AB (180, 340)
goal = (520, 320)
human_position = (300, 300)
human_radius = 3

robot_position = start
robot_path = [robot_position]  # ロボットの経路を記録するリスト


class KalmanFilterNode:
    def __init__(self):
        rospy.init_node('kalman_filter_node', anonymous=True)

        # ROS Parameters
        self.prev_time = rospy.get_time()  # 前回の時間
        self.dt = 0.2  # 時間差、初期値
        self.R_std = 1  # 観測ノイズの標準偏差

        self.theta_p = -1 # 角度が取得できなかった時用
        self.judge = 0 # 判定ライン用
        self.danger = 0 # ロボットがいる方向に曲がってくる可能性(三段階評価) ❍❍❍   

        self.timer = 0 # ロボット移動用のタイマー

        self.check = 0 # ルート変更の判定用(ロボットの縦移動に使用)
        self.moving_count = 0
        self.y_moving=0
        self.y_count=0
    

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
        self.robot_position = ()

        # ゴールの設置
        self.goal_position=(520, 320)

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

            """
            # **************ロボットの行動規則**************
            # ルート変更に関して
            # 判定ライン通過時に、右に曲がりそうかどうかを判定する条件文
            # 進行方向は固定であるとする(切り返しは考慮しない)
            # cv上では右がx軸正方向、下がy軸正方向
            if (1.0 <= future_y < 1.7 and self.judge ==0):
                self.judge += 1
                rospy.loginfo(f"*****Passed through judgment line 1*****")
                rospy.loginfo(f"Time：{time_str}")
                future_direction = math.atan2(self.x[3, 0], self.x[4, 0]) # 速度方向を計算
                future_direction_deg = (math.degrees(future_direction)+270)%360 # 右をx軸正、下をy軸正とする
                rospy.loginfo(f"Direction of travel：{future_direction_deg:3.1f}[°]")
                if  0<= future_direction_deg <= 90:
                    rospy.loginfo("Driving Route: A →  A")
                elif 90 < future_direction_deg <= 180 or 0.1 <= future_x < 0.66 : # add_7.30
                    self.danger +=1
                    rospy.loginfo("Driving Route: A →  B")
                else:
                    rospy.loginfo("進行方向が反対です")
                    self.judge -=1
                #rospy.loginfo(f"危険度:{self.danger}") # 確認用

            if (0.3 <= future_y < 1.0 and self.judge ==1):
                self.judge += 1
                rospy.loginfo(f"*****Passed through judgment line 2*****")
                rospy.loginfo(f"Time：{time_str}")
                future_direction = math.atan2(self.x[3, 0], self.x[4, 0]) # 速度方向を計算
                future_direction_deg = (math.degrees(future_direction)+270)%360 # 右をx軸正、下をy軸正とする
                rospy.loginfo(f"Direction of travel：{future_direction_deg:3.1f}[°]")
                if 0 <= future_direction_deg <= 90:
                    if self.danger == 1:
                        rospy.loginfo("Driving Route: B →  B")
                    else:
                        rospy.loginfo("Driving Route: A →  A")
                elif 90 < future_direction_deg <= 180 or 0.1 <= future_x < 0.66: # add_7.30
                    self.danger +=1
                    if self.danger == 2:
                        rospy.loginfo("Driving Route: B →  C")
                    else:
                        self.danger +=1
                        rospy.loginfo("Driving Route: A →  C")
                #rospy.loginfo(f"危険度:{self.danger}") # 確認用

            if (future_y < 0.3 and self.judge ==2):
                self.judge += 1
                rospy.loginfo(f"*****Passed through judgment line 3*****")
                rospy.loginfo(f"Time：{time_str}")
                future_direction = math.atan2(self.x[3, 0], self.x[4, 0]) # 速度方向を計算
                future_direction_deg = (math.degrees(future_direction)+270)%360 # 右をx軸正、下をy軸正とする
                rospy.loginfo(f"Direction of travel：{future_direction_deg:3.1f}[°]")
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
                        self.danger +=1
                        rospy.loginfo("Driving Route: B →  D")
                    else:
                        self.danger +=2
                        rospy.loginfo("Driving Route: A →  D")
                #rospy.loginfo(f"危険度:{self.danger}") # 確認用
            """

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
        """
        end_point_AB = (int(3.2*cv +width/2), int(-1*(-0.5)*cv +height/2))
        start_point_BC = (int(-1.5*cv +width/2), int(-1*(-1)*cv+height/2))
        end_point_BC = (int(3.2*cv +width/2), int(-1*(-1)*cv +height/2))
        start_point_CD = (int(-1.5*cv +width/2), int(-1*(-1.5)*cv+height/2))
        end_point_CD = (int(3.2*cv +width/2), int(-1*(-1.5)*cv +height/2))
        color = (100, 100, 100)
        thickness = 2
        segment_length = 10 # 点線のセグメントの長さ
        """

        # ゴールの設置
        cv2.circle(image, self.goal_position, 8, (255, 0, 255), -1)

        # ロボットの描画
        if(self.danger!=self.check):
            self.moving_count = self.danger*step # stepは、なんステップでロボットがルート移動を完了するかを表す。
            #rospy.loginfo(f"変わった：{self.momoving_count}")

        #rospy.loginfo(f"y_count：{self.y_count}, moving_count{self.moving_count}")
        if(self.y_count < self.moving_count):
            self.y_count += 1
            self.y_moving = int(self.danger*40.0*self.y_count/self.moving_count)
            #rospy.loginfo(f"移動距離(縦)：{self.y_moving}")


        # ロボットの軌跡

        # 四角形の位置を計算
        current_x = start_point_AB[0] + int(self.timer * x_distance * cv / times)
        current_y = start_point_AB[1] - 15 + self.y_moving - 8

        # 現在の位置をリストに追加
        positions.append((current_x, current_y))

        # ロボットの移動軌跡を描画
        for i in range(1, len(positions)):
            # 前の位置と現在の位置を取得
            prev_pos = positions[i-1]
            curr_pos = positions[i]
            # 前の位置と現在の位置を結ぶ線を描画
            cv2.line(image, prev_pos, curr_pos, (255, 255, 200), 1, cv2.LINE_AA)
            cv2.circle(image, curr_pos, 2, (255, 255, 200), -1)

        # ロボットの描画
        if(mode==1):
            cv2.rectangle(image, (start_point_AB[0]+int(self.timer*x_distance*cv/times), start_point_AB[1]-15+self.y_moving), (start_point_AB[0]+int(self.timer*x_distance*cv/times)+30, start_point_AB[1]-15-15+self.y_moving), (200, 100, 0), cv2.FILLED, cv2.LINE_AA)
            cv2.circle(image, (start_point_AB[0]+int(self.timer*x_distance*cv/times)+7, start_point_AB[1]-10+self.y_moving), 4, (200, 200, 200), -1)
            cv2.circle(image, (start_point_AB[0]+int(self.timer*x_distance*cv/times)+23, start_point_AB[1]-10+self.y_moving), 4, (200, 200, 200), -1)

            self.robot_position=(start_point_AB[0]+int(self.timer*x_distance*cv/times)+30, start_point_AB[1]-15-15//2+self.y_moving) # 軌道描画用
        
        else:
            cv2.rectangle(image, (start_point_AB[0]+int(self.timer*x_distance*cv/times), start_point_AB[1]-15), (start_point_AB[0]+int(self.timer*x_distance*cv/times)+30, start_point_AB[1]-15-15), (200, 100, 0), cv2.FILLED, cv2.LINE_AA)
            cv2.circle(image, (start_point_AB[0]+int(self.timer*x_distance*cv/times)+7, start_point_AB[1]-10), 4, (200, 200, 200), -1)
            cv2.circle(image, (start_point_AB[0]+int(self.timer*x_distance*cv/times)+23, start_point_AB[1]-10), 4, (200, 200, 200), -1)

            self.robot_position=(start_point_AB[0]+int(self.timer*x_distance*cv/times)+30, start_point_AB[1]-15-15//2) # 軌道描画用
        
        self.check = self.danger

        # 軌道生成・表示----------------------------------------------------------------------------
        for step in range(600):  # ゴールに到達するために十分なステップ数を設定
            """
            # 人の位置をランダムに移動
            new_human_position = (human_position[0] + random.randint(-1, 1), human_position[1] + random.randint(-1, 1))

            # 人がグリッド内に留まるように調整
            new_human_position = (max(0, min(grid_size - 1, new_human_position[0])),
                                  max(0, min(grid_size - 1, new_human_position[1])))

            # 障害物の上に移動しないようにする
            human_position = get_valid_human_position(new_human_position, grid)
            """

            # ロボットの経路を計画
            path = astar(grid, self.robot_position, goal, human_position, human_radius)

            if path:
                print(f"ステップ {step}: 経路が見つかりました: {path}")
                # ロボットを複数ステップ進める
                steps_to_advance = min(3, len(path) - 1)  # 一度に進むステップ数を増やす
                robot_position = path[steps_to_advance]

                robot_path.append(robot_position)  # ロボットの位置を経路に追加
            else:
                print(f"ステップ {step}: 経路が見つかりませんでした、再計画中...")

        #cv2.line(image, self.robot_position, self.goal_position, (0, 165, 255), 2)




        # -------------------------------------------------------------------------------------------(8/20)

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