#Problem 3: Calculate right and left Wheel Velocities in rpm

#Given
wheel_space = 0.156     #in mm center-to-center
wheel_diameter = 0.065   #in m

def calc_wheel_vel(vel):
    vel_l = float(str(round(vel, 2)))
    vel_R = float(str(round(vel, 2)))
    wheel_vel = [vel_l, vel_R]
    return wheel_vel

wheel_vel = calc_wheel_vel(1)
