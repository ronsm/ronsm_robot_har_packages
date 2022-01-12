import requests

from time import sleep

from ralt_map_helper import RALTMapHelper

class OpenHABHelper():
    def __init__(self, url):
        self.url = url
        self.item_states = False
        self.prev_item_states = False
        self.room = 'wait'

        self.rmh = RALTMapHelper()
        self.rooms = self.rmh.get_rooms()

        self.init = True

    """ 
    input: n/a
    output: data from openhab

    Retrieves a list of all items and their states from OpenHAB.
    """
    def retrieve(self):
        try:
            response = requests.get(self.url)
            items = response.json()
            self.item_states = items
            if self.init:
                self.prev_item_states = self.item_states
                self.init = False
            return self.process()
        except:
            print('Unable to connect to OpenHAB endpoint.')
            return None

    """ 
    input: n/a
    output: name of a room where last sensor event took place

    Iterates over all OpenHAB items to check which ones have updated in the last few seconds.
    Returns the name of the room for latest sensor event, taken from their OpenHAB tags.
    """
    def process(self):
        candidates = []
        for i in range(0, len(self.item_states)):
            if self.item_states[i]['state'] != self.prev_item_states[i]['state']:
                candidates.append(self.item_states[i]['label'])

        rooms = []
        if candidates:
            for candidate in candidates:
                for i in range(0, len(self.item_states)):
                    if self.item_states[i]['label'] == candidate:
                        for tag in self.item_states[i]['tags']:
                            if tag in self.rooms:
                                rooms.append(tag)

        if rooms:
            return rooms[0]
        else:
            return None

    """ 
    input: n/a
    output: n/a

    Keeps the OpenHAB helper alive, checking for new data every t seconds.
    """
    def spin(self):
        while(True):
            self.retrieve()
            room = self.process()
            if room != None:
                self.room = room
                # print(self.room)
            self.prev_item_states = self.item_states
            sleep(10)

    """ 
    Class Function
    
    Returns the current room.
    """
    def get_room(self):
        return self.room

if __name__ == '__main__':
    ohh = OpenHABHelper('https://caregrouphwu%40icloud.com:G00drobot6@home.myopenhab.org:443/rest/items')
    ohh.spin()