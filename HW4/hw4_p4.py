#Problem 4: Applying kalman filter to the accelration values along X axis

import math
import csv
from hw4_p1 import calc_mean_var
import matplotlib.pyplot as plt

def kalman_filter(data):
    upd_x = [0]
    state_estimate = [0]
    P = [0]
    upd_P = [0]
    Q = R = 0.1
    K = [0]
    t = [0]

    k = 0
    while k < 201:
        if k > 0: 
            upd_x.append(state_estimate[k-1])
            upd_P.append(P[k-1] + Q)
            K.append(upd_P[k]/(upd_P[k] + R))
            state_estimate.append(upd_x[k] + (K[k] * (data[k] - upd_x[k])))
            P.append( (1 - K[k]) * upd_P[k])
            t.append(0.025*k)   
        k = k + 1
    return state_estimate, t

if __name__ == "__main__":
    csv_path = 'RBE3002_D20_homework_4 - Vnav_Data.csv'
    file = csv.reader(open(csv_path, 'r'))
    data = calc_mean_var(file)
    acc_x = data[6]
    new_data = kalman_filter(acc_x)
    filtered_acc_x = new_data[0]
    t = new_data[1]

    #plotting graphs
    plt.plot(t, acc_x[0:201])
    plt.plot(t, filtered_acc_x)
    plt.show()

