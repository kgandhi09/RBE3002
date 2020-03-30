#Problem 2: Calculating Mean and Variance

import random
import csv
import numpy as np

#Note: Please change the path to equivalent destination
csv_path = 'HW1/hw1_p2_data.csv'

random_int = random.randint(10,30)

randomList = []
#Generating a list of random integers
for i in range(0, random_int):
    randomList.append(random.uniform(1,30))

#writing randomly generated list to a csv file
with open(csv_path, 'wb') as f:
    writer = csv.writer(f)
    for val in randomList:
        writer.writerow([val])


#reading numbers from csv file
file = csv.reader(open(csv_path, 'r'))
readList = []
for row in file:
    readList.append(row[0])

#converting the list of strings read from csv to int
for i in range(len(readList)):
    readList[i] = float(readList[i])

#Calculating the mean and variance of randomly read numbers from the list
def calc_mean_var(list):
    mean_list = np.mean(list)
    var_list = np.var(list)

    mean_list = float(str(round(mean_list, 2)))
    var_list = float(str(round(var_list, 2)))
    print(mean_list, var_list)

calc_mean_var(readList)
