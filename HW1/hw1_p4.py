#Problem 4: "Invalid Velocity Request!"

from math import pi
test_value = 0.21

#Given
wheel_space = 0.156     #in mm center-to-center
wheel_diameter = 0.065   #in m
wheel_radius = wheel_diameter/2     #in m

#building up on the code from problem 3
def calc_wheel_vel(vel):

    # if the translational velocity is lesser than max velocity which is 0.22 m/s, then move
    if vel < 0.22:
        ang_vel_l = vel*60/(2*pi*wheel_radius)        
        ang_vel_r = ang_vel_l            

        ang_vel_l = float(str(round(ang_vel_l, 2)))           
        ang_vel_r = float(str(round(ang_vel_r, 2)))
        
        wheel_vel = [ang_vel_l, ang_vel_r]
        print(wheel_vel)
        return wheel_vel

    #else print the message
    else:
        print("Invalid Velocity Request!")

if __name__ == "__main__":
    wheel_vel = calc_wheel_vel(test_value)
    