#! /usr/bin/env python3

import pickle
import rospkg

rospack = rospkg.RosPack()
rel_path = rospack.get_path('robot_har_mln')

train_db_h = [[[('involves_event', 'Kettle'), ('involves_event', 'DrinkwareCabinet')], 'PreparingDrink'], [[('involves_event', 'Oven'), ('involves_event', 'DinnerwareCabinet')], 'Cooking']]
path = rel_path + '/src/DBs/' + 'global' + '_DB_h.p'
pickle.dump(train_db_h, open(path, 'wb'))

train_db_s =  [[[('involves_event', 'Kettle'), ('involves_event', 'DrinkwareCabinet')], 'PreparingDrink'], [[('involves_event', 'Oven'), ('involves_event', 'DinnerwareCabinet')], 'Cooking'], [[('involves_percept', 'Cup'), ('involves_event', 'DrinkwareCabinet')], 'PreparingDrink']]
path = rel_path + '/src/DBs/' + 'global' + '_DB_s.p'
pickle.dump(train_db_s, open(path, 'wb'))