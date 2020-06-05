#Problem  5:
import math
import csv
from hw4_p1 import calc_mean_var
from hw4_p4 import kalman_filter
from hw4_p3 import calc_vel_dist
import matplotlib.pyplot as plt

if __name__ == "__main__":
    csv_path = 'RBE3002_D20_homework_4 - Vnav_Data.csv'
    file = csv.reader(open(csv_path, 'r'))
    data = calc_mean_var(file)
    acc_x = data[6]
    new_acc_x = acc_x

    # Removing the DC Bias
    for i in range(len(new_acc_x)):
        new_acc_x[i] = new_acc_x[i] - data[0]

    #Using kalman filter on the list with zero mean
    filtered_data = kalman_filter(new_acc_x)
    filtered_acc_x = filtered_data[0]
    t = filtered_data[1]

    #plotting graphs
    plt.plot(t, new_acc_x[0:201])
    plt.plot(t, filtered_acc_x)

    vel_dist = calc_vel_dist(filtered_acc_x)

    plt.show()
    