#Problem 3: Calculate right and left Wheel Velocities in rpm

from math import pi
test_value = 0.05     #in m/s

#Given
wheel_space = 0.156     #in mm center-to-center
wheel_diameter = 0.065   #in m
wheel_radius = wheel_diameter/2   #in m

#Assuming the robot is moving in straight motion
def calc_wheel_vel(vel):
    #Converting from linear velocity to angular velocity
    ang_vel_l = (vel*60 )/(2*pi*wheel_radius)        
    #Since robot is moving in straight, left wheel velocity = right wheel velocity
    ang_vel_r = ang_vel_l              

    #rounding up to two decimal places
    ang_vel_l = float(str(round(ang_vel_l, 2)))           
    ang_vel_r = float(str(round(ang_vel_r, 2)))

    wheel_vel = [ang_vel_l, ang_vel_r]
    print(wheel_vel)
    return wheel_vel

if __name__ == '__main__':
    wheel_vel = calc_wheel_vel(test_value)
    