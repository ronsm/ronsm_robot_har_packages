#!/usr/bin/env python3
import rospkg

from log import Log

class LabelLinker(object):
    def __init__(self, dataset):
        self.id = 'label_linker'

        self.logger = Log(self.id)

        self.dataset = dataset

        self.rospack = rospkg.RosPack()

        self.load_label_links()

        self.logger.log_great('Ready.')

    def load_label_links(self):
        path = self.rospack.get_path('robot_har_dialogue_system') + '/src/data/' + self.dataset + '/label_links.txt'
        print(path)
        try:
            file = open(path, 'r')
            lines = file.readlines()
        except:
            print('Missing label links (label_links.txt) file.')

        self.link_dict = {}
        self.link_descriptions = {}

        for line in lines:
            line = line.rstrip('\n')
            splits = line.split(':')
            
            if splits[1] not in self.link_dict:
                self.link_dict[splits[1]] = list()
                self.link_descriptions[splits[1]] = splits[2]
            
            self.link_dict[splits[1]].append(splits[0])

    def get_model_label(self, ADL_label):
        for key, labels in self.link_dict.items():
            for label in labels:
                if label == ADL_label:
                    return key

    def get_ADL_labels(self, model_label):
        for key, labels in self.link_dict.items():
            if key == model_label:
                return labels

    def get_model_label_description(self, model_label):
        for key, description in self.link_descriptions.items():
            if key == model_label:
                return description