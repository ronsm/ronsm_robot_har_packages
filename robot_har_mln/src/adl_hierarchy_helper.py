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

        self.load_rooms()

        self.logger.log_great('Ready.')

    def load_rooms(self):
        self.rooms = []
        for key, value in self.adls.items():
            for item in value:
                for inner_key, inner_value in item.items():
                    try:
                        len(inner_value[0]['rooms']) < 1
                    except TypeError:
                        log = 'Error with YAML file KBs/adls.yaml, ADL: ' + inner_key + ' has no rooms specified.'
                        self.logger.log_warn(log)
                        quit()

                    self.rooms.append((inner_key, inner_value[0]['rooms']))

    def get_parents(self):
        parents = []
        for p in self.adls:
            parents.append(p)

        return parents

    def get_children(self, valid_room=None):
        children = []
        for key, value in self.adls.items():
            for item in value:
                for inner_key, inner_value in item.items():
                    children.append(inner_key)
    
        if valid_room != None:
            valid_children = []
            for child in children:
                valid = self.is_valid_in_room(child, valid_room)
                if valid:
                    valid_children.append(child)
            children = valid_children
        
        return children

    def get_parent(self, child):
        for p in self.adls:
            for c in self.adls[p]:
                for key, value in c.items():
                    if key == child:
                        return p

    def get_all(self):
        joint = self.get_parents() + self.get_children()
        joint = list(dict.fromkeys(joint)) # removes duplicates caused by parent sharing name with a child
        return joint

    def is_multiroom(self, adl):
        for room in self.rooms:
            if room[0] == adl:
                if len(room[1]) == 1:
                    if room[1][0] == 'all':
                        return True
                    else:
                        return False
                elif len(room[1]) > 1:
                    return True

    def is_valid_in_room(self, adl, check_room):
        for room in self.rooms:
            if room[0] == adl:
                if check_room in room[1]:
                    return True
                else:
                    return False

    def get_rooms(self, adl):
        for room in self.rooms:
            if room[0] == adl:
                return room[1]

if __name__ == '__main__':
    ahh = ADLHierarchyHelper('/home/ronsm/catkin_ws/src/ronsm_robot_har_packages/robot_har_mln')
    ahh.logger.log_mini_header('[TESTING] BEGIN UNIT TESTING')
    ahh.logger.log_mini_header('Testing: get_parents()')
    print(ahh.get_parents())
    ahh.logger.log_mini_header('Testing: get_children()')
    print(ahh.get_children())
    ahh.logger.log_mini_header('Testing: get_children(valid_room=Kitchen)')
    print(ahh.get_children(valid_room='Kitchen'))
    ahh.logger.log_mini_header('Testing: get_parent(Showering)')
    print(ahh.get_parent('Showering'))
    ahh.logger.log_mini_header('Testing: get_all()')
    print(ahh.get_all())
    ahh.logger.log_mini_header('Testing: is_multiroom(Cooking) - should be False')
    print(ahh.is_multiroom('Cooking'))
    ahh.logger.log_mini_header('Testing: is_multiroom(Cleaning) - should be True')
    print(ahh.is_multiroom('Cleaning'))
    ahh.logger.log_mini_header('Testing: is_multiroom(UsingSmartphone) - should be True')
    print(ahh.is_multiroom('UsingSmartphone'))
    ahh.logger.log_mini_header('Testing: get_rooms(Cleaning) - Working')
    print(ahh.get_rooms('Cleaning'))
    ahh.logger.log_mini_header('Testing: get_rooms(UsingSmartphone) - Cleaning')
    print(ahh.get_rooms('UsingSmartphone'))
    ahh.logger.log_mini_header('[TESTING] END UNIT TESTING')