#! /usr/bin/env python3

import pprint

from log import Log

class TrainDBConsistencyHelper():
    def __init__(self, rel_path):
        self.id = 'train_db_consistency_helper'

        self.logger = Log(self.id)

        self.rel_path = rel_path

        self.load_dbs()
        self.reduce()

    def load_dbs(self):
        path = self.rel_path + '/src/DBs/' + 'global' + '_DB_h.txt'

        global_db_h = open(path, 'r')

        samples_h = []
        sample = []
        for line in global_db_h:
            l = line.rstrip('\n')
            sample.append(l)
            if l == '---':
                sample.pop()
                samples_h.append(sample)
                sample = []

        path = self.rel_path + '/src/DBs/' + 'global' + '_DB_s.txt'

        global_db_s = open(path, 'r')

        samples_s = []
        sample = []
        for line in global_db_s:
            l = line.rstrip('\n')
            sample.append(l)
            if l == '---':
                sample.pop()
                samples_s.append(sample)
                sample = []

        self.samples_h = samples_h
        self.samples_s = samples_s

    def reduce(self):
        adls_h = []
        adls_s = []

        for sample in self.samples_h:
            adl = None
            events = []
            for p in sample:
                if p[0:5] == 'class':
                    adl = p
                    sample.remove(p)
                elif len(p) > 14 and p[0:14] == 'involves_event':
                    events.append(p)
            adls_h.append((adl, events))

        for sample in self.samples_s:
            adl = None
            events = []
            for p in sample:
                if p[0:5] == 'class':
                    adl = p
                    sample.remove(p)
                elif len(p) > 14 and p[0:14] == 'involves_event':
                    events.append(p)
            adls_s.append((adl, events))

        # create empty disctionaries with keys as classes, values as empty arrays
        adls_h_dict = {}
        adls_s_dict = {}

        for adl in adls_h:
            adls_h_dict[adl[0]] = []

        for adl in adls_s:
            adls_s_dict[adl[0]] = []
        
        # for each sample, append preds to adl classes
        for adl in adls_h:
            adls_h_dict[adl[0]] = adls_h_dict[adl[0]] + adl[1]

        for adl in adls_s:
            adls_s_dict[adl[0]] = adls_s_dict[adl[0]] + adl[1]

        # remove duplicate preds from adl classes
        for key, value in adls_h_dict.items():
            adls_h_dict[key] = set(value)

        for key, value in adls_s_dict.items():
            adls_s_dict[key] = set(value)

        self.adls_h_dict = adls_h_dict
        self.adls_s_dict = adls_s_dict

    def is_consistent(self, adl_h, adl_s, event):
        consistent_h = True
        consistent_s = True

        adl_h = 'class(S,' + adl_h + ')'
        adl_s = 'class(S,' + adl_s + ')'

        try:
            valid_h = self.adls_h_dict[adl_h]

            if event not in valid_h:
                consistent_s = False
        except KeyError:
            self.logger.log_warn('Tried to look up consistency for an ADL which has no training samples (H).')
            consistent_h = False

        try:
            valid_s = self.adls_s_dict[adl_s]

            if event not in valid_s:
                consistent_s = False
        except KeyError:
            self.logger.log_warn('Tried to look up consistency for an ADL which has no training samples (S).')
            consistent_s = False

        return consistent_h, consistent_s

if __name__ == '__main__':
    tdch = TrainDBConsistencyHelper('/home/ronsm/catkin_ws/src/ronsm_robot_har_packages/robot_har_mln')
    print(tdch.is_consistent('class(S,Bed)', 'class(S,Sleeping)', 'involves_event(S,Fridge)'))
    print(tdch.is_consistent('class(S,FoodDrink)', 'class(S,TalkingOnPhone)', 'involves_event(S,Fridge)'))