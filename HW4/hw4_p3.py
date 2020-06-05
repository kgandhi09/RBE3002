#Problem 3
import math
import csv
import numpy as np
from hw4_p1 import calc_mean_var


def calc_vel_dist(acc_x):
    delta_t = 0.025   #in seconds

    vel = [0]
    dist = [0]


    for k in range(len(acc_x)):
        if k > 0:
            vel.append(vel[k-1] + delta_t*acc_x[k-1])
            dist.append(dist[k-1] + delta_t*vel[k-1])


    final_vel = vel[-1]
    final_dist = dist[-1]

    print  'Final Velocity: ' + str(final_vel) + ' m/s' + '\nFinal Dist: ' + str(final_dist) + ' m'
    return (final_vel, final_dist, vel, dist)

if __name__ == "__main__":
    csv_path = 'RBE3002_D20_homework_4 - Vnav_Data.csv'
    file = csv.reader(open(csv_path, 'r'))
    data = calc_mean_var(file)
    acc_x = data[6]
    new_acc_x = acc_x

    # Part a: Calculating distance travelled and velocity towards x axis
    print('\nProblem 3 part a')
    part_a = calc_vel_dist(acc_x)

    # Part b: Calculating distance travelled and velocity towards x axis with zero mean
    # Making the data Zero mean
    for i in range(len(new_acc_x)):
        new_acc_x[i] = acc_x[i] - data[0]

    print('\nProblem 3 part b')
    part_b = calc_vel_dist(new_acc_x)

    print new_acc_x
