#Problem 6: Calculating the pose

import csv
import matplotlib.pyplot as plt
import math

def localize(file):
    readlist = []
    
    angles = []
    ranges = []

    coord = []

    for row in file:
        readlist.append(row)
    
    for i in range(len(readlist)):
        if i > 0:
            angles.append(float(readlist[i][0]))
            ranges.append(float(readlist[i][1]))

    #converting to cartesian coordinates
    for i in (range(len(angles))):
        x = ranges[i]*math.cos(angles[i])
        y = ranges[i]*math.sin(angles[i])
        coord.append((x,y))

    coord.remove(coord[0])

    pos_coord = []
    neg_coord = []

    #closest value to y=0:
    min_pos = 0
    max_neg = 0

    #Finding the closest value to y=0
    for i in range(len(coord)):
        if coord[i][1] >= 0:
            pos_coord.append(coord[i])

        if coord[i][1] < 0 :
            neg_coord.append(coord[i])

    if min(pos_coord) > (max(neg_coord))*1:
        pose_x = min(pos_coord)[0]
    else:
        pose_x = max(neg_coord)[0]

    theta = 0
    for i in range(len(coord)):
        if coord[i][0] == pose_x:
            theta = i

    theta = -(180-angles[theta])

    pose_y = 0
    print pose_x, pose_y, theta
        
    

if __name__ == "__main__":
    csv_path = 'RBE3002_D20_HW4 _Sensor_Data.csv'
    file = csv.reader(open(csv_path, 'r'))
    
    localize(file)