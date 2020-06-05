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

    # coord.remove(coord[0])

    x_list = []
    y_list = []

  
    for i in range(len(coord)):
        x_list.append(coord[i][0])
        y_list.append(coord[i][1])

    #Sorting x and y values
    new_x = []
    new_y = []


    for i in range(len(y_list)):
        if y_list[i] - y_list[i-1] < 0.000000000000001 and i > 0:
            new_y.append(y_list[i])

    plt.scatter(angles, x_list)
    plt.scatter(angles, y_list)
    plt.show()


    '''
    Answer Part B: 
    By stdying the graph, intersections can be easily seen which can be
    interpreted as the corners of the walls,

    THe corners of the walls are: (-7.63, -7.606), (-3.629-3.689)

    '''

if __name__ == "__main__":
    csv_path = 'RBE3002_D20_HW4 _Sensor_Data.csv'
    file = csv.reader(open(csv_path, 'r'))
    
    localize(file)