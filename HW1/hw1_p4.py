#Problem 4: "Invalid Velocity Request!"

#Given
wheel_space = 0.156     #in mm center-to-center
wheel_diameter = 0.065   #in m

def calc_wheel_vel(vel):

    if(vel <= 0.22):
        vel_l = float(str(round(vel, 2)))
        vel_R = float(str(round(vel, 2)))
        wheel_vel = [vel_l, vel_R]
        return wheel_vel
    else:
        print("Invalid Velocity Request!")

wheel_vel = calc_wheel_vel(1)