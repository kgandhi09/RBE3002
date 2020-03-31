#Problem 2: Calculating Mean and Variance
import csv
import math

#Note: Please change the path to equivalent destination
csv_path = 'HW1/hw1_p2_data.csv'

randomList = []

#reading numbers from csv file
file = csv.reader(open(csv_path, 'r'))
for row in file:
    randomList.append(row[0])

#converting the list of strings read from csv to int
for i in range(len(randomList)):
    randomList[i] = float(randomList[i])


#Calculating the mean and variance of randomly read numbers from the list
#Approach 1: Using standard calculations
def calc_mean_var1(list):
    sum = 0

    #calculating mean
    for i in range(len(list)):
        sum = sum + list[i]

    mean_list = sum/len(list)
    mean_list = float(str(round(mean_list, 2)))

    #calculating Variance
    var_sum = 0
    for j in range(len(list)):
        var_sum = var_sum + math.pow((list[j] - mean_list), 2)

    var_list = var_sum/len(list)
    var_list = float(str(round(var_list, 2)))

    print(mean_list, var_list)


#Calculating the mean and variance of randomly read numbers from the list
#Approach 2: Using built-in library numpy

import numpy as np

def calc_mean_var2(list):
    mean_list = np.mean(list)
    var_list = np.var(list)

    mean_list = float(str(round(mean_list, 2)))
    var_list = float(str(round(var_list, 2)))
    print(mean_list, var_list)

calc_mean_var1(randomList)
calc_mean_var2(randomList)