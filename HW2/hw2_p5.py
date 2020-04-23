#Problem 5

import math
import numpy as np
from matplotlib import pyplot as plt
from hw2_p3 import traj_generate_3
from hw2_p4 import traj_generate_4

def calc_euc_dist(trajectory):
    euc_dist = []
    count = 0
    while count < len(trajectory):
        dist = math.sqrt( (trajectory[count][0])**2 + (trajectory[count][1])**2 )
        if count > 0:
            dist = math.sqrt( (trajectory[count][0] - trajectory[count-1][0])**2 + (trajectory[count][1] - trajectory[count-1][1])**2)
        euc_dist.append(dist)
        count = count + 1
    return euc_dist


if __name__ == "__main__":
    path_plan = [[15.3, 0.188, 0.22, 0, 0, 1.5709], 
                    [15.3, 0.22, 0.188, 2, 0, -1.5709], 
                    [9.1, 0.22, 0.22, 4, 0, 1.5709], 
                    [26.5, 0.15, 0.15, 4, 2, -3.14159], 
                    [ 9.1, 0.22, 0.22, 0, 2, -1.5709] ]

    trajectory_3 = traj_generate_3(path_plan)
    euc_dist_3 = calc_euc_dist(trajectory_3)

    trajectory_4 = traj_generate_4(path_plan)
    euc_dist_4 = calc_euc_dist(trajectory_4)

    print("\nEucladian Distances from problem 3 - \n")
    print(euc_dist_3)

    print("\nEucladian Distances from problem 4 - \n")
    print(euc_dist_4)