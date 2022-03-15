#! /usr/bin/env python3
import yaml

from log import Log

class ADLHierarchyHelper():
    def __init__(self, rel_path):
        self.id = 'adl_hierarchy_helper'
        self.logger = Log(self.id)

        self.rel_path = rel_path

        adl_path = self.rel_path + '/src/KBs/adls.yaml'
        with open(adl_path, 'r') as stream:
            try:
                self.adls = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)

        self.logger.log_great('Ready.')

    def get_parents(self):
        parents = []
        for p in self.adls:
            parents.append(p)

        return parents

    def get_children(self):
        children = []
        for p in self.adls:
            children = children + self.adls[p]
        
        return children

    def get_parent(self, child):
        for p in self.adls:
            for c in self.adls[p]:
                if c == child:
                    return p

    def get_all(self):
        return self.get_parents + self.get_children

if __name__ == '__main__':
    adlhh = ADLHierarchyHelper('/home/ronsm/catkin_ws/src/ronsm_robot_har_packages/robot_har_mln')
    adlhh.get_parents()
    adlhh.get_children()
    adlhh.get_parent('Showering')