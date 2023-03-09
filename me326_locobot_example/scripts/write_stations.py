import numpy as np
import os
import pickle



station_locations_xy_rad = np.matrix([[-0.5, 1.0, 0.15],
[0.5, 1.25, 0.2],
[1.0, 0.0, 0.3],
[0.0, -0.75, 0.15]])

target_config_rows_rgby_cols_station_ABC = np.matrix([[2,1,0],
                                                       [1,0,1],
                                                       [1,1,1],
                                                       [1,1,0]])
robot_1_colors = np.matrix([1,1,0,0]).T #RGBY
robot_2_colors = np.matrix([0,0,1,1]).T #RGBY

info = [station_locations_xy_rad, target_config_rows_rgby_cols_station_ABC, robot_1_colors, robot_2_colors]

dict_labels = ["station_locations","target_config","robot_1_colors","robot_2_colors"]
info_dictionary = dict(zip(dict_labels,info))
print(info_dictionary)

# open file in binary mode
with open('data.pickle', 'wb') as f:
    # save data to file
    pickle.dump(info_dictionary, f)
