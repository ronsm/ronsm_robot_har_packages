#!/usr/bin/env python3

class RALTMapHelper(object):
    def __init__(self):
        self.rooms = ['kitchen', 'lounge', 'dining', 'bedroom', 'bathroom', 'hall']
        self.points = {
            'kitchen': [0.0, 0.0],
            'lounge': [0.0, 0.0],
            'dining': [0.0, 0.0],
            'bedroom': [0.0, 0.0],
            'bathroom': [0.0, 0.0],
            'hall': [0.0, 0.0],
        }

    """ 
    Class Function
    
    Returns the default position for a named room.
    """
    def get_point(self, room):
        try:
            return self.points[room]
        except KeyError:
            print('[ERROR] Invalid key provided to RALTMapHelper.get_point(self, room).')

    """ 
    Class Function
    
    Returns the list of rooms.
    """
    def get_rooms(self):
        return self.rooms