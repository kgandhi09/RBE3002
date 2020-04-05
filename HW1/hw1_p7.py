#Problem 7: average of longitudes and lattitudes

import numpy as np
readpath = "HW1/gps_data.txt" 

#reading the gps_data file
def lat_long_mean(fileread):
    gps_data = open(fileread, "r")
    gps_list = []
    longitude_list = []
    lattitude_list = []
    dir_list1 = []
    dir_list2 = []

    for line in gps_data:
        stripped_line = line.strip()
        line_list = stripped_line.split()
        gps_list.append(line_list)

    i = 0

    while True:
        temp_gps_list = gps_list[i][0].split(',')
        lattitude_list.append(float(temp_gps_list[2]))
        longitude_list.append(float(temp_gps_list[4]))
        dir_list1.append(temp_gps_list[3])
        dir_list2.append(temp_gps_list[5])
        i += 1
        if i > len(gps_list) - 1:
            break

    lat_mean = float(str(round(np.mean(lattitude_list), 2)))
    long_mean = float(str(round(np.mean(longitude_list), 2)))

    final_list = [lat_mean, 'N', long_mean, 'W']
    return final_list

mean_list = lat_long_mean(readpath)
print(mean_list)