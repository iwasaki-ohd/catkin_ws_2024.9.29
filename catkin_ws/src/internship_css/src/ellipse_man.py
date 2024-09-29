#!/usr/bin/env python
# coding: UTF-8

import rospy
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import cv2
import math
import matplotlib.pyplot as plt
from scipy.stats import multivariate_normal
from matplotlib.patches import Ellipse

t = 2  # 何秒後を予測するかを決定する変数
cv = 70  # スケーリング用変数(cv2の表示のために使用)

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

        self.theta_p = -1  # 角度が取得できなかった時用
        self.judge = 0  # 判定ライン用
        self.denger = 0  # ロボットがいる方向に曲がってくる可能性(三段階評価) ❍❍❍

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
        # self.Q = np.diag([0.49, 0.49, 0.49, 5.175, 5.175, 5.175])  # システムノイズの共分散行列
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

        y = np.array([measurement[0], measurement[1], self.x[2, 0] + angle_diff]).reshape(-1, 1) - np.dot(
            self.H, self.x)
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

            if float(data_str[4]) < 0 and self.theta_p >= 0:  # 取得角度が-1の時、角度が取得できなかったことを表す。このときは、前回の予測角度を現在の取得角度として扱う。
                theta = self.theta_p
                rospy.loginfo(f"角度が取得できなかったため前回の予測角度を再利用します。正規化{(theta)},θ={(data_str[4])}")

            # フィルターを更新
            self.update([x, y, theta])

            # 予測された状態をパブリッシュ
            predicted_msg = PoseStamped()
            predicted_msg.header.stamp = rospy.Time.now()
            predicted_msg.pose.position.x = self.x[0, 0]
            predicted_msg.pose.position.y = self.x[1, 0]
            predicted_msg.pose.orientation.z = self.x[2, 0]
            self.predicted_pub.publish(predicted_msg)

            rospy.loginfo(f"時刻：{time_str}")
            rospy.loginfo("Current Position: x={}, y={}, θ={}".format(x, y, theta))
            rospy.loginfo(f"Current predicted Position: x={self.x[0, 0]}, y={self.x[1, 0]}, θ={self.x[2, 0]}")

            # t秒後の予測位置を計算
            future_x = self.x[0, 0] + self.x[3, 0] * t
            future_y = self.x[1, 0] + self.x[4, 0] * t
            future_theta = self.x[2, 0] + self.x[5, 0] * t
            rospy.loginfo(f"{t}秒後のPredicted Position: x={future_x}, y={future_y}, θ={future_theta}")

            future_direction = math.atan2(self.x[3, 0], self.x[4, 0])  # 速度方向を計算
            future_direction_deg = (math.degrees(future_direction) + 270) % 360  # 右をx軸正、下をy軸正とする
            rospy.loginfo(f"進行方向：{future_direction_deg}[°]")

            # 判定ライン通過時に、右に曲がりそうかどうかを判定する条件分
            # 進行方向は固定であるとする(切り返しは考慮しない)
            # cv上では右がx軸正方向、下がy軸正方向
            if (1.0 <= future_y < 1.7 and self.judge == 0):
                self.judge += 1
                rospy.loginfo(f"*****判定ライン1通過*****")
                future_direction = math.atan2(self.x[3, 0], self.x[4, 0])  # 速度方向を計算
                future_direction_deg = (math.degrees(future_direction) + 270) % 360  # 右をx軸正、下をy軸正とする
                rospy.loginfo(f"進行方向：{future_direction_deg}[°]")
                if 0 <= future_direction_deg <= 90:
                    rospy.loginfo("大丈夫そう     ❍ ❍ ❍")
                elif 90 < future_direction_deg <= 180:
                    self.denger += 1
                    rospy.loginfo("右っぽい       ● ❍ ❍")
                else:
                    rospy.loginfo("進行方向が反対です")
                    self.judge -= 1

            if (0.3 <= future_y < 1.0 and self.judge == 1):
                self.judge += 1
                rospy.loginfo(f"*****判定ライン2通過*****")
                future_direction = math.atan2(self.x[3, 0], self.x[4, 0])  # 速度方向を計算
                future_direction_deg = (math.degrees(future_direction) + 270) % 360  # 右をx軸正、下をy軸正とする
                rospy.loginfo(f"進行方向：{future_direction_deg}[°]")
                if 0 <= future_direction_deg <= 90:
                    if self.denger == 1:
                        rospy.loginfo("大丈夫そう     ● ❍ ❍")
                    else:
                        rospy.loginfo("大丈夫そう     ❍ ❍ ❍")
                elif 90 < future_direction_deg <= 180:
                    self.denger += 1
                    rospy.loginfo("右っぽい       ● ● ❍")
            if (future_y < 0.3 and self.judge == 2):
                self.judge += 1
                rospy.loginfo(f"*****判定ライン3通過*****")
                future_direction = math.atan2(self.x[3, 0], self.x[4, 0])  # 速度方向を計算
                future_direction_deg = (math.degrees(future_direction) + 270) % 360  # 右をx軸正、下をy軸正とする
                rospy.loginfo(f"進行方向：{future_direction_deg}[°]")
                if 0 <= future_direction_deg <= 90:
                    if self.denger == 2:
                        rospy.loginfo("大丈夫そう     ● ● ❍")
                    elif self.denger == 1:
                        rospy.loginfo("大丈夫そう     ● ❍ ❍")
                    else:
                        rospy.loginfo("大丈夫そう     ❍ ❍ ❍")
                elif 90 < future_direction_deg <= 180:
                    self.denger += 1
                    rospy.loginfo("右来ます       ● ● ●")

            rospy.loginfo(f"--------------------------------------------------------------")
            self.theta_p = self.x[2, 0]  # 取得角度が-1の時に使われる

            # 予測位置の計算
            future_positions = self.calculate_future_positions()

            # OpenCVを使用して可視化
            self.visualize_position(future_positions)

        except IndexError:
            rospy.logerr("Invalid sensor data format: {}".format(data.data))
        except ValueError:
            rospy.logerr("Invalid sensor data values: {}".format(data.data))

    def calculate_future_positions(self):
        future_positions = []
        for i in range(-10, 11):  # t秒後に存在しうる予測位置の範囲を計算
            future_x = self.x[0, 0] + self.x[3, 0] * t * i
            future_y = self.x[1, 0] + self.x[4, 0] * t * i
            future_theta = (self.x[2, 0] + self.x[5, 0] * t * i) % 360
            future_positions.append((future_x, future_y, future_theta))
        return future_positions

    def visualize_position(self, future_positions):
        # 予測位置を可視化する
        plt.figure(figsize=(8, 8))
        ax = plt.gca()
        for x, y, theta in future_positions:
            ellipse = Ellipse((x, y), width=cv, height=cv, angle=theta, fill=False, color='red')
            ax.add_patch(ellipse)
            plt.scatter(x, y, color='red', s=5)
        plt.xlim(0, width)
        plt.ylim(0, height)
        plt.grid(True)
        plt.gca().set_aspect('equal', adjustable='box')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Predicted Positions')
        plt.show()

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = KalmanFilterNode()
        node.run()
    except rospy.ROSInterruptException:
        pass

