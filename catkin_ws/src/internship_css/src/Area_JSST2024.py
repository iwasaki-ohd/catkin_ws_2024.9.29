#!/usr/bin/env python
# coding: UTF-8


import rospy
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import cv2
import math

t = 2 # 何秒後を予測するかを決定する変数
cv = 70 # スケーリング用変数(cv2の表示のために使用)

# cv領域のサイズ
width = 500
height = 500

class KalmanFilterNode:
    def __init__(self):
        rospy.init_node('kalman_filter_node', anonymous=True)

        # ROS Parameters
        self.prev_time = rospy.get_time()  # 前回の時間
        self.dt = 0.3  # 時間差、初期値
        self.R_std = 1.0  # 観測ノイズの標準偏差

        self.theta_p = -1 # 角度が取得できなかった時用
        self.judge = 0 # 判定ライン用
        self.denger = 0 # ロボットがいる方向に曲がってくる可能性(三段階評価) ❍❍❍        

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
        self.x = np.zeros((6, 1))  # 初期状態
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
            time_str = data_str[0]
            id_val = int(data_str[1])
            x = float(data_str[2])
            y = float(data_str[3])
            theta = float(data_str[4]) % 360  # 角度を0〜360に正規化

            if float(data_str[4]) < 0 and self.theta_p >= 0: # 取得角度が-1の時、角度が取得できなかったことを表す。このときは、前回の予測角度を現在の取得角度として扱う。
                theta = self.theta_p

            # フィルターを更新
            self.update([x, y, theta])

            # 予測された状態をパブリッシュ
            predicted_msg = PoseStamped()
            predicted_msg.header.stamp = rospy.Time.now()
            predicted_msg.pose.position.x = self.x[0, 0]
            predicted_msg.pose.position.y = self.x[1, 0]
            predicted_msg.pose.orientation.z = self.x[2, 0]
            self.predicted_pub.publish(predicted_msg)

            # t秒後の予測位置を表示
            future_x = self.x[0, 0] + self.x[3, 0] * t
            future_y = self.x[1, 0] + self.x[4, 0] * t
            future_theta = self.x[2, 0] + self.x[5, 0] * t

            future_direction = math.atan2(self.x[3, 0], self.x[4, 0]) # 速度方向を計算
            future_direction_deg = (math.degrees(future_direction)+270)%360 # 右をx軸正、下をy軸正とする
            
            if (1.0 <= future_y < 1.7 and self.judge ==0):
                self.judge += 1
                rospy.loginfo(f"*****Passed through judgment line 1*****")
                rospy.loginfo(f"Time：{time_str}")
                future_direction = math.atan2(self.x[3, 0], self.x[4, 0]) # 速度方向を計算
                future_direction_deg = (math.degrees(future_direction)+270)%360 # 右をx軸正、下をy軸正とする

                if  0<= future_direction_deg <= 90:
                    rospy.loginfo("Driving Route: A →  A")
                elif 90 < future_direction_deg <= 180:
                    self.denger +=1
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

                if 0 <= future_direction_deg <= 90:
                    if self.denger == 1:
                        rospy.loginfo("Driving Route: B →  B")
                    else:
                        rospy.loginfo("Driving Route: A →  A")
                elif 90 < future_direction_deg <= 180:
                    self.denger +=1
                    if self.denger == 2:
                        rospy.loginfo("Driving Route: B →  C")
                    else:
                        rospy.loginfo("Driving Route: A →  C")
            if (future_y < 0.3 and self.judge ==2):
                self.judge += 1
                rospy.loginfo(f"*****Passed through judgment line 3*****")
                rospy.loginfo(f"Time：{time_str}")
                future_direction = math.atan2(self.x[3, 0], self.x[4, 0]) # 速度方向を計算
                future_direction_deg = (math.degrees(future_direction)+270)%360 # 右をx軸正、下をy軸正とする

                if 0 <= future_direction_deg <= 90:
                    if self.denger == 1:
                        rospy.loginfo("Driving Route: B →  B")
                    elif self.denger == 2:
                        rospy.loginfo("Driving Route: C →  C")
                    else:
                        rospy.loginfo("Driving Route: A →  A")
                elif 90 < future_direction_deg <= 180:
                    self.denger +=1
                    if self.denger == 2:
                        rospy.loginfo("Driving Route: B →  C")
                    elif self.denger == 3:
                        rospy.loginfo("Driving Route: C →  D")
                    else:
                        rospy.loginfo("Driving Route: A →  D")
                else:
                    rospy.loginfo("進行方向が反対です")
                    self.judge -=1

            rospy.loginfo(f"predicted position: x = {future_x}, y = {future_y}, theta = {future_theta}")
            rospy.loginfo(f"velocity direction：{future_direction_deg}")

            # 軌跡の保存
            self.observation_trajectory.append((int(x * cv + width / 2), int(-y * cv + height / 2)))
            self.prediction_trajectory.append((int(future_x * cv + width / 2), int(-future_y * cv + height / 2)))

            self.theta_p = theta

            # 可視化
            self.visualize_position()

        except (IndexError, ValueError) as e:
            rospy.logwarn(f"Invalid data received: {data_str}")
            return

    def visualize_position(self):
        # カレントポジションと次の予測位置を可視化
        image = np.zeros((width, height, 3), dtype=np.uint8)  # 仮の画像を作成

        # 観測値の軌跡を描画
        if hasattr(self, 'observation_trajectory'):
            for i in range(1, len(self.observation_trajectory)):
                cv2.line(image, self.observation_trajectory[i - 1], self.observation_trajectory[i], (50, 255, 50), 1)

        # 予測値の軌跡を描画
        if hasattr(self, 'prediction_trajectory'):
            for i in range(1, len(self.prediction_trajectory)):
                cv2.line(image, self.prediction_trajectory[i - 1], self.prediction_trajectory[i], (50, 50, 255), 1)

        # カレントポジションを描画
        cv2.circle(image, (self.observation_trajectory[-1][0], self.observation_trajectory[-1][1]), 5, (0, 255, 0), -1)
        # 次の予測位置を描画
        cv2.circle(image, (self.prediction_trajectory[-1][0], self.prediction_trajectory[-1][1]), 5, (0, 0, 255), -1)
        
        # 研究室のマップを表示
        map_coordinates = [((-1.5,-1), (3.2, -1)),
               ((-1.5,0), (0.1, 0)),
               ((0.1, 0), (0.1, 2.4)),
               ((0.1,2.4), (-1.5, 2.4)),
               ((-1.5,3.2), (3.2, 3.2)),
               ((3.2,3.2), (3.2, -0.4)),
               ((3.2,0), (2, 0)),
               ((2,0), (2, 2.4)),
               ((2,2.4), (3.2, 2.4))]
               
        for coord_pair in map_coordinates:
            pt1, pt2 = coord_pair
            # 座標を整数に変換してから線を引く
            pt1 = (int(pt1[0]*cv+width/2), int(-1*pt1[1]*cv+height/2))  # 座標のスケーリング
            pt2 = (int(pt2[0]*cv+width/2), int(-1*pt2[1]*cv+height/2))  # 座標のスケーリング
            cv2.line(image, pt1, pt2, (255, 255,255), 2)
        
        # 判定ライン
        cv2.line(image, (int(0.1*cv +width/2), int(-1*0.3*cv+height/2)), (int(2*cv +width/2), int(-1*0.3*cv +height/2)), (100, 100,255), 2) # 2
        cv2.line(image, (int(0.1*cv +width/2), int(-1*1.0*cv+height/2)), (int(2*cv +width/2), int(-1*1.0*cv +height/2)), (100, 255,255), 2) # 1
        cv2.line(image, (int(0.1*cv +width/2), int(-1*1.7*cv+height/2)), (int(2*cv +width/2), int(-1*1.7*cv +height/2)), (255, 255,100), 2) # 0

        # 共分散楕円を描画
        self.plot_covariance_ellipse(image)

        # 画像を表示
        cv2.imshow("Position Visualization", image)
        cv2.waitKey(1)

    def plot_covariance_ellipse(self, image):
        # 共分散行列から楕円をプロット
        cov = self.P[0:2, 0:2]  # x, y 位置の共分散行列
        mean = self.x[0:2, 0]  # x, y 位置の平均

        eigenvalues, eigenvectors = np.linalg.eig(cov)
        order = eigenvalues.argsort()[::-1]
        eigenvalues = eigenvalues[order]
        eigenvectors = eigenvectors[:, order]

        angle = np.arctan2(*eigenvectors[:, 0][::-1])
        angle = np.degrees(angle)

        ell_width, ell_height = 2 * np.sqrt(eigenvalues)

        center = (int(mean[0] * cv + width // 2), int(-mean[1] * cv + height // 2))
        axes = (int(ell_width * cv / 2), int(ell_height * cv / 2))

        cv2.ellipse(image, center, axes, angle, 0, 360, (0, 0, 255), 2)

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
