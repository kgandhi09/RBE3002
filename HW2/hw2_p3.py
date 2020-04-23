#Problem 3

import math
import numpy as np
from matplotlib import pyplot as plt

def traj_generate(path_plan):
    base = 0.156
    pose = []
    pose_list = []
    init_time = 0
    delta_time = 0.1 
    i=0
    
    while i < len(path_plan):
        j=0
        final_time = path_plan[i][0]
        left_vel = path_plan[i][1]
        right_vel = path_plan[i][2]
        init_x = path_plan[i][3]
        init_y = path_plan[i][4]
        init_theta = path_plan[i][5]

        if right_vel - left_vel == 0:
            R = 0
            omega = 0
        else:
            omega = (right_vel - left_vel)/base
            R = ((right_vel + left_vel)/(right_vel-left_vel))*(base/2)

        curr_x = init_x 
        curr_y = init_y
        curr_theta = init_theta
        curr_time = init_time
        
        pose = [curr_x, curr_y, curr_theta, curr_time]
        pose_list.append(pose)

        while j < final_time - 0.1:
            if right_vel - left_vel == 0:
                dist = right_vel*delta_time
                curr_y = curr_y + dist*math.sin(init_theta)
                curr_x = curr_x + dist*math.cos(init_theta)
                curr_time = curr_time + delta_time
                pose = [curr_x, curr_y, curr_theta, curr_time]
                pose_list.append(pose)
            else:
                curr_x = curr_x + R*((math.sin(omega*delta_time + curr_theta)) - math.sin(curr_theta))
                curr_y = curr_y - R*((math.cos(omega*delta_time + curr_theta)) - math.cos(curr_theta))
                curr_time = curr_time + delta_time
                curr_theta = omega*delta_time + curr_theta
                pose = [curr_x, curr_y, curr_theta, curr_time]
                pose_list.append(pose)
           
            j = j + delta_time

        i = i + 1

    print_list(pose_list)
    count = 0

    while count < len(pose_list):
        plt.plot(pose_list[0], pose_list[1])
        count = count + 1
    plt.show()
    

def print_list(list):
    for j in list:
        print j


if __name__ == "__main__":
    path_plan = [[15.3, 0.188, 0.22, 0, 0, 1.5709], 
                [15.3, 0.22, 0.188, 2, 0, -1.5709], 
                [9.1, 0.22, 0.22, 4, 0, 1.5709], 
                [26.5, 0.15, 0.15, 4, 2, -3.14159], 
                [ 9.1, 0.22, 0.22, 0, 2, -1.5709] ]
                
    traj_generate(path_plan)
    


