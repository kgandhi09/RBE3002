#Problem 5: Specify Velocity, Direction and ICC

#Given
wheel_space = 0.156     #in mm center-to-center
wheel_diameter = 0.065   #in m

F = 'F'         #forward
R = 'R'         #reverse

def calc_wheel_vel(vel, dir, *icc):
    if not icc:
        if(vel <= 0.22 and dir == 'F'):
            vel_l = float(str(round(vel, 2)))
            vel_R = float(str(round(vel, 2)))
            wheel_vel = [vel_l, vel_R]
            return wheel_vel
        if(vel <= 0.22 and dir == 'R'):
            vel_l = -float(str(round(vel, 2)))
            vel_R = -float(str(round(vel, 2)))
            wheel_vel = [vel_l, vel_R]
            return wheel_vel
        if vel > 0.22:
            print("Invalid Velocity Request!")
       
    if icc:
        icc = icc[0]
        if vel <= 0.22 and dir == 'F':
            w_omega = icc*vel
            vel_l = vel - w_omega*(wheel_space/2)
            vel_r = vel + w_omega*(wheel_space/2)
            wheel_vel = [vel_l, vel_r]
            return wheel_vel
        if vel <= 0.22 and dir == 'R':
            w_omega = icc*vel
            vel_l = vel + w_omega*(wheel_space/2)
            vel_r = vel - w_omega*(wheel_space/2)
            wheel_vel = [vel_l, vel_r]
            return wheel_vel
        if vel > 0.22:
            print("Invalid Velocity Request!")


wheel_vel = calc_wheel_vel(0.22, R, 0.1)
print(wheel_vel)

