#! /usr/bin/env python3
import yaml
import os
import pickle

from log import Log

class ADLHelper():
    def __init__(self, rel_path, reset):
        self.id = 'adl_helper'
        self.logger = Log(self.id)

        self.rel_path = rel_path

        self.room_path = self.rel_path + '/src/KBs/rooms.pickle'

        adl_path = self.rel_path + '/src/KBs/adls.yaml'
        with open(adl_path, 'r') as stream:
            try:
                self.adls = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)

        if reset:
            self.create_room_file()

        self.load_room_file()

        print(self.room_dict)

        self.logger.log_great('Ready.')

    def create_room_file(self):
        try:
            os.remove(self.room_path)
        except FileNotFoundError:
            pass
        
        room_dict = {}

        for adl in self.get_adls():
            room_dict[adl] = []

        pickle.dump(room_dict, open(self.room_path, 'wb'))

    def load_room_file(self):
        self.room_dict = pickle.load(open(self.room_path, 'rb'))

    def save_room_file(self):
        pickle.dump(self.room_dict, open(self.room_path, 'wb'))
        self.load_room_file()

    def update_rooms(self, adl, rooms):
        self.room_dict[adl] = self.room_dict[adl] + rooms
        self.room_dict[adl] = set(self.room_dict[adl])
        self.room_dict[adl] = list(self.room_dict[adl])

        self.save_room_file()

    def get_adls(self, valid_room=None):
        if valid_room == None:
            return self.adls
        else:
            room_adls = []
            for key, value in self.room_dict.items():
                if valid_room in value:
                    room_adls.append(key)
            return room_adls

    def is_multiroom(self, adl):
        rooms = self.room_dict[adl]
        if len(rooms) == 0:
            return True
        elif len(rooms) == 1:
            return False
        elif len(rooms) > 1:
            return True

    def is_valid_in_room(self, adl, check_room):
        rooms = self.room_dict[adl]
        if check_room in rooms:
            return True
        else:
            return False

    def get_rooms(self, adl):
        return self.room_dict[adl]

if __name__ == '__main__':
    ahh = ADLHelper('/home/ronsm/catkin_ws/src/ronsm_robot_har_packages/robot_har_mln', reset=True)
    ahh.logger.log_mini_header('[TESTING] BEGIN UNIT TESTING')
    ahh.logger.log_mini_header('Testing: get_adls()')
    print(ahh.get_adls())
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
    ahh.logger.log_mini_header('Testing: update_rooms(Cooking, [rooms])')
    ahh.update_rooms('Cooking', ['bedroom'])
    ahh.update_rooms('Cooking', ['bedroom'])
    ahh.update_rooms('Cooking', ['bedroom', 'kitchen'])
    ahh.update_rooms('Cooking', ['bedroom', 'kitchen', 'lounge'])
    ahh.logger.log_mini_header('Testing: get_adls(valid_room=bedroom)')
    print(ahh.get_adls(valid_room='bedroom'))