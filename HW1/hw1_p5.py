#Problem 5: Specify Velocity, Direction and ICC

from math import pi

#Given
wheel_space = 0.156     #in m center-to-center
wheel_diameter = 0.065   #in m
wheel_radius = wheel_diameter/2     #in m

F = 'F'         #forward
R = 'R'         #reverse

def calc_wheel_vel(vel, dir, *icc):
    if vel < 0.22:
        #if there is icc mentioned, which means robot is following a curved path
        if icc:
            _icc = icc[0]
            omega = vel/_icc
            vel_l = omega*(_icc + wheel_space/2)                    #calculating left_wheel velocity in m/s
            vel_r = omega*(_icc - wheel_space/2)                    #calculating right_wheel velocity in m/s
            
            ang_vel_l = (vel_l*60 )/(2*pi*wheel_radius)             #converting left_wheel velocity to rpm
            ang_vel_r = (vel_r*60 )/(2*pi*wheel_radius)             #converting right_wheel velocity to rpm

            #handling the negative sign according to the direction
            if dir == F:
                ang_vel_l = float(str(round(ang_vel_l, 2)))           
                ang_vel_r = float(str(round(ang_vel_r, 2)))
            
            elif dir == R:
                ang_vel_l = -float(str(round(ang_vel_l, 2)))           
                ang_vel_r = -float(str(round(ang_vel_r, 2))) 
            
        #if robot is moving in straight motion
        else:
            ang_vel_l = vel*60/(2*pi*wheel_radius)        
            ang_vel_r = ang_vel_l      

            #handling the negative sign according to the direction
            if dir == F:
                ang_vel_l = float(str(round(ang_vel_l, 2)))           
                ang_vel_r = float(str(round(ang_vel_r, 2)))
            
            elif dir == R:
                ang_vel_l = -float(str(round(ang_vel_l, 2)))           
                ang_vel_r = -float(str(round(ang_vel_r, 2)))

        wheel_vel = [ang_vel_l, ang_vel_r]
        print(wheel_vel)
        return wheel_vel

    else:
        print('Invalid Velocity Request!')


if __name__ == "__main__":
    wheel_vel = calc_wheel_vel(0.21, F, 0.5)


