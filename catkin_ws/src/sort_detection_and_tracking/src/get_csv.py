#!/usr/bin/env python

# 2.10
# 移動データをcsv形式で出力

import rospy
from std_msgs.msg import String
import csv

class DataLogger:
    def __init__(self):
        rospy.init_node('data_logger', anonymous=True)
        self.csv_file = 'integration_data.csv'
        self.fieldnames = ['ID', 'wc_x', 'wc_y', 'arc']  # CSVの列のヘッダー
        self.setup_csv()

    def setup_csv(self):
        with open(self.csv_file, mode='w') as file:
            writer = csv.DictWriter(file, fieldnames=self.fieldnames)
            writer.writeheader()

    def callback(self, data):
        with open(self.csv_file, mode='a') as file:
            writer = csv.DictWriter(file, fieldnames=self.fieldnames)

            # 受信したデータを分割してリストに格納
            data_list = data.data.split(',')

            # 受信したデータの数がフィールドの数と異なる場合は、適切に処理する
            data_dict = {key: value for key, value in zip(self.fieldnames, data_list[:len(self.fieldnames)])}

            # 行を書き込む
            writer.writerow(data_dict)

    def listener(self):
        rospy.Subscriber('/integration_data', String, self.callback)
        rospy.spin()

if __name__ == '__main__':
    logger = DataLogger()
    logger.listener()

