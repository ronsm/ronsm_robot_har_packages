#!/usr/bin/env python3

# standard libraries
import numpy as np
import pandas as pd
from time import perf_counter, sleep, strftime
import csv
import rospkg
import os

# internal classes
from log import Log

# standard messages
# none

# custom messages
# none

# constants and parameters
# none

class CSVTools():
    def __init__(self):
        # set up logger
        self.id = 'csv_tools'
        self.logger = Log(self.id)
        self.logger.startup_msg()

        # set up ROS
        rospack = rospkg.RosPack()
        rel_path = rospack.get_path('robot_har_rasa')
        self.csv_folder = rel_path + '/src/logs'

        # init
        csv_folder_exists = os.path.exists(self.csv_folder)
        if not csv_folder_exists:
            self.logger.log_warn('ASR log folder does not exist! Will try to create it...')
            os.makedirs(self.csv_folder)
            self.logger.log_great('Created ASR log folder.')

        self.create_csv()

        # ready
        self.logger.log_great('Ready.')

    def create_csv(self):
        date_time = strftime("%Y%m%d-%H%M%S")
        self.csv_filename = self.csv_folder + '/annotation_' + date_time + '.csv'

        msg = 'The ASR log file for this session is ' + self.csv_filename
        self.logger.log(msg)

        with open(self.csv_filename, 'w', newline='') as fd:
            writer = csv.writer(fd)

            header = ['time', 'utterance']

            writer.writerow(header)

    def save_utterance(self, utterance):
        date_time = strftime("%Y%m%d-%H%M%S")
        row = [date_time, utterance]

        with open(self.csv_filename, 'a', newline='') as fd:
            writer = csv.writer(fd)
            writer.writerow(row)
    