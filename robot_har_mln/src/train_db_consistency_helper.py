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

        self.samples_s = samples_s

    def reduce(self):
        adls_s = []

        for sample in self.samples_s:
            adl = None
            events = []
            for p in sample:
                if p[0:5] == 'class':
                    adl = p
                    sample.remove(p)
                elif len(p) > 8 and p[0:8] == 'involves':
                    events.append(p)
            adls_s.append((adl, events))

        # create empty dictionaries with keys as classes, values as empty arrays
        adls_s_dict = {}

        for adl in adls_s:
            adls_s_dict[adl[0]] = []
        
        # for each sample, append preds to adl classes
        for adl in adls_s:
            adls_s_dict[adl[0]] = adls_s_dict[adl[0]] + adl[1]

        # remove duplicate preds from adl classes
        for key, value in adls_s_dict.items():
            adls_s_dict[key] = set(value)

        self.adls_s_dict = adls_s_dict

    def is_consistent(self, adl, event):
        consistent = True

        adl = 'class(S,' + adl + ')'

        try:
            valid = self.adls_s_dict[adl]

            if event not in valid:
                consistent = False
        except KeyError:
            self.logger.log_warn('Tried to look up consistency for an ADL which has no training samples.')
            consistent = False

        return consistent

if __name__ == '__main__':
    tdch = TrainDBConsistencyHelper('/home/ronsm/catkin_ws/src/ronsm_robot_har_packages/robot_har_mln')