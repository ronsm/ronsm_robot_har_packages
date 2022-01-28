#! /usr/bin/env python3

import pickle
import rospkg

rospack = rospkg.RosPack()
rel_path = rospack.get_path('robot_har_mln')

train_db_h = [["header"]]
path = rel_path + '/src/DBs/' + 'global' + '_DB_h.p'
pickle.dump(train_db_h, open(path, 'wb'))

train_db_s = [["header"]]
path = rel_path + '/src/DBs/' + 'global' + '_DB_s.p'
pickle.dump(train_db_s, open(path, 'wb'))