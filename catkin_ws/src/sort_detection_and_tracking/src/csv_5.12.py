#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import csv

class IntegrationDataLogger:
    def __init__(self):
        rospy.init_node('integration_data_logger', anonymous=True)
        self.subscriber = rospy.Subscriber('/integration_data', String, self.callback)
        self.csv_file = open('integration_data.csv', 'w')
        self.csv_writer = csv.writer(self.csv_file)
    
    def callback(self, data):
        # Split the received data by comma
        data_list = data.data.split(',')
        
        # Write the data to CSV file
        self.csv_writer.writerow(data_list)
        self.csv_file.flush()  # Ensure data is written immediately
        #self.csv_file.write('\n')
        
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        data_logger = IntegrationDataLogger()
        data_logger.run()
    except rospy.ROSInterruptException:
        pass

