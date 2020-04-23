def final_pose_calc(path_plan):
    base = 0.156
    
     
    delta_time = path_plan[0]
    left_vel = path_plan[1]
    right_vel = path_plan[2]
    init_x = path_plan[3]
    init_y = path_plan[4]
    init_theta = path_plan[5]
    if right_vel - left_vel == 0:
        R = math.isinf(float("-inf"))
        omega = 0
    else:  
        omega = (right_vel - left_vel)/base
        R = ((right_vel + left_vel)/(right_vel-left_vel))*(base/2)

    ICC = [(init_x - (R*math.sin(init_theta))), (init_y + (R*math.cos(init_theta)))]

    A = np.array([ [ math.cos(omega*delta_time), -math.sin(omega*delta_time), 0], 
                   [ math.sin(omega*delta_time), math.cos(omega*delta_time), 0], 
                   [0, 0, 1] ])

    B = np.array([ [init_x - ICC[0]],
                   [init_y - ICC[1]],
                   [init_theta] ])

    C = np.array([ [ICC[0]],
                   [ICC[1]],
                   [omega*delta_time] ])

    D = A.dot(B)

    final = sum_matrix(D, C)
    print(final)

def sum_matrix(X,Y):
    result = [[0,0,0], 
              [0,0,0], 
              [0,0,0]] 

    for i in range(len(X)):    
# iterate through columns 
       for j in range(len(X[0])): 
           result[i][j] = X[i][j] + Y[i][j] 
  
    # for r in result: 
    #     # print(r) 

    return result