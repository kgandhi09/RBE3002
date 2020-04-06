#Problem 7: average of longitudes and lattitudes

import numpy as np
readpath = "HW1/gps_data.txt" 


def lat_long_mean(fileread):
    #reading the gps_data file
    gps_data = open(fileread, "r")

    gps_list = []
    longitude_list = []
    lattitude_list = []
    dir_list1 = []
    dir_list2 = []

    #converting the text read from file to list: gps_list
    for line in gps_data:
        stripped_line = line.strip()
        line_list = stripped_line.split()
        gps_list.append(line_list)

    i = 0

    #splitting the entire line to read out only direction, lattitude and longitude values
    while True:
        temp_gps_list = gps_list[i][0].split(',')
        lattitude_list.append(float(temp_gps_list[2]))
        longitude_list.append(float(temp_gps_list[4]))
        dir_list1.append(temp_gps_list[3])
        dir_list2.append(temp_gps_list[5])
        i += 1
        if i > len(gps_list) - 1:
            break

    #calculating the mean for all the lattitude and longitude values
    lat_mean = str(round(np.mean(lattitude_list), 2))
    long_mean = str(round(np.mean(longitude_list), 2))

    #formatting for lattitude values
    dd_lat = lat_mean[0:2]
    mm_lat = lat_mean[2:4]
    ss_lat = lat_mean[5:7]
    lat_mean = dd_lat + " " + mm_lat + " " + ss_lat 

    #formatting for longitude values
    dd_long = long_mean[0:2]
    mm_long = long_mean[2:4]
    ss_long = long_mean[5:7]
    long_mean = dd_long + " " + mm_long + " " + ss_long 

    final_gps_list = [lat_mean, 'N', long_mean, 'W']
    print final_gps_list

if __name__ == "__main__":
    lat_long_mean(readpath)