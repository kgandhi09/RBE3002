#Problem 1: Calculating mean and variances of acceleration along x y and z axis

#Reading from the file 

import csv 
import math
import numpy as np

def calc_mean_var(file):

    readlist = []
    for row in file:
        readlist.append(row)

    acc_x = []
    acc_y = []
    acc_z = []

    for i in range(len(readlist)):
        if i > 0:
            acc_x.append(float(readlist[i][0]))
            acc_y.append(float(readlist[i][1]))
            acc_z.append(float(readlist[i][2]))


    #Calculating Mean along X axis 
    mean_x = np.mean(acc_x)
    var_x = np.var(acc_x)

    #Calculating Mean along Y axis 
    mean_y = np.mean(acc_y)
    var_y = np.var(acc_y)

    #Calculating Mean along X axis 
    mean_z = np.mean(acc_z)
    var_z = np.var(acc_z)

    return [mean_x, var_x, mean_y, var_y, mean_z, var_z, acc_x, acc_y, acc_z]


if __name__ == "__main__":
    csv_path = 'RBE3002_D20_homework_4 - Vnav_Data.csv'
    file = csv.reader(open(csv_path, 'r'))
    data = calc_mean_var(file)

    print 'X Axis: Mean = ' +  str(data[0]) + ', Variance = ' + str(data[1])
    print 'Y Axis: Mean = ' +  str(data[2]) + ', Variance = ' + str(data[3])
    print 'Z Axis: Mean = ' +  str(data[4]) + ', Variance = ' + str(data[5]) 