#! /usr/bin/env python3

from log import Log

class KnownDomainsHelper():
    def __init__(self, rel_path):
        self.id = 'known_domains_helper'

        self.logger = Log(self.id)

        self.rel_path = rel_path

        self.load_dbs()
        self.count()

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

    def count(self):
        self.known_events = set()
        self.known_times = set()
        self.known_adls = set()

        for sample in self.samples_s:
            for p in sample:
                splits = p.split(',')
                splits[1] = splits[1].rstrip(')')

                if p[0:5] == 'class':
                    self.known_adls.add(splits[1])

                if p[0:8] == 'involves':
                    self.known_events.add(splits[1])

                if p[0:6] == 'occurs':
                    self.known_times.add(splits[1])

        return self.known_events, self.known_times, self.known_adls

if __name__ == '__main__':
    kdh = KnownDomainsHelper('/home/ronsm/catkin_ws/src/ronsm_robot_har_packages/robot_har_mln')