#!/usr/bin/env python2

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from std_msgs.msg import String
from math import sqrt, pi, atan2, sin, cos

class Lab2:

    
    def __init__(self):
        """
        Class constructor
        """
        self.px = 0
        self.py = 0
        self.pth = 0
        # REQUIRED CREDIT
        # Initialize node, name it 'lab2'
        # TODO
        rospy.init_node('lab2', anonymous=True)
        # ### Tell ROS that this node publishes Twist messages on the '/cmd_vel' topic
        # # TODO
        # print('vel publisherr')
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # ### Tell ROS that this node subscribes to Odometry messages on the '/odom' topic
        # ### When a message is received, call self.update_odometry
        # # TODO
        # print('odom subscriber')
        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.update_odometry)
        # ### Tell ROS that this node subscribes to PoseStamped messages on the '/move_base_simple/goal' topic
        # ### When a message is received, call self.go_to
        # # TODO
        # print('pose subscriber')
        self.pose_subscriber = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.go_to)

    def traj_generate(self, path_plan):
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
                    curr_y = curr_y + dist*sin(init_theta)
                    curr_x = curr_x + dist*cos(init_theta)
                    curr_time = curr_time + delta_time
                    pose = [curr_x, curr_y, curr_theta, curr_time]
                    pose_list.append(pose)
                else:
                    curr_x = curr_x + R*((sin(omega*delta_time + curr_theta)) - sin(curr_theta))
                    curr_y = curr_y - R*((cos(omega*delta_time + curr_theta)) - cos(curr_theta))
                    curr_time = curr_time + delta_time
                    curr_theta = omega*delta_time + curr_theta
                    pose = [curr_x, curr_y, curr_theta, curr_time]
                    pose_list.append(pose)
            
                j = j + delta_time

            i = i + 1

        return pose_list 

    def send_speed(self, linear_speed, angular_speed):
        """
        Sends the speeds to the motors.
        :param linear_speed  [float] [m/s]   The forward linear speed.
        :param angular_speed [float] [rad/s] The angular speed for rotating around the body center.
        """
        # REQUIRED CREDIT
        # Make a new Twist message
        # TODO                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
        self.msg_cmd_vel = Twist()
        #Linear velocity
        self.msg_cmd_vel.linear.x=linear_speed
        self.msg_cmd_vel.linear.y=0.0
        self.msg_cmd_vel.linear.z=0.0
        #Angular Velocity
        self.msg_cmd_vel.angular.x = 0.0
        self.msg_cmd_vel.angular.y = 0.0

        if angular_speed < 0:
            clockwise = True
        else:
            clockwise = False

        if clockwise:
            self.msg_cmd_vel.angular.z = -abs(angular_speed)
        else:
            self.msg_cmd_vel.angular.z = abs(angular_speed)


        # Publish the message
        # TODO
        self.velocity_publisher.publish(self.msg_cmd_vel)


    
        
    def drive(self, distance, linear_speed):
        """
        Drives the robot in a straight line.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The forward linear speed.
        """
        # REQUIRED CREDIT
        current_distance = 0
        init_time = rospy.get_time()
        while current_distance < distance:
            time_now = rospy.get_time()
            current_distance = linear_speed*(time_now-init_time)
            Lab2.send_speed(self, linear_speed,0)
        self.msg_cmd_vel.linear.x = 0
        self.velocity_publisher.publish(self.msg_cmd_vel)
        



    def rotate(self, angle, aspeed):
        """
        Rotates the robot around the body center by the given angle.
        :param angle         [float] [rad]   The distance to cover.
        :param angular_speed [float] [rad/s] The angular speed.
        """

        # REQUIRED CREDIT
        init_time = rospy.Time.now().to_sec()
        current_angle = 0

        while current_angle < abs(angle):
            Lab2.send_speed(self,0,aspeed)
            time = rospy.Time.now().to_sec()
            if aspeed > 0:
                current_angle = aspeed*(time - init_time)
            else:
                current_angle = -aspeed*(time - init_time)
        self.msg_cmd_vel.angular.z = 0
        self.velocity_publisher.publish(self.msg_cmd_vel)


    def go_to(self, path_plan):
        """
        Calls rotate(), drive(), and rotate() to attain a given pose.
        This method is a callback bound to a Subscriber.
        :param msg [PoseStamped] The target pose.
        """
        #REQUIRED CREDIT
        
        trajectory = Lab2().traj_generate(path_plan)

        count = 0
        while count < len(trajectory):
            dist = sqrt( (trajectory[count][0])**2 + (trajectory[count][1])**2)
            ort = trajectory[count][2]
            if count > 0:
                prev_x = trajectory[count-1][0]
                prev_y = trajectory[count-1][1]
                prev_theta = trajectory[count-1][2]
                curr_x = trajectory[count][0]
                curr_y = trajectory[count][1]
                curr_theta = trajectory[count][2]

                dist = sqrt( (curr_x - prev_x)**2 + (curr_y - prev_y)**2)
                ort = curr_theta - prev_theta

            print dist, ort

            if dist != 0:
                Lab2().drive(dist, 0.1)

            if ort < 0:
                Lab2().rotate(ort, -0.1)
            else:
                Lab2().rotate(ort, 0.1)
            count = count + 1


    def update_odometry(self, msg):
        """
        Updates the current pose of the robot.
        This method is a callback bound to a Subscriber.
        :param msg [Odometry] The current odometry information.
        """
        # REQUIRED CREDIT
        # TODO
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y
        quat_orig=msg.pose.pose.orientation
        quat_list=[quat_orig.x,quat_orig.y,quat_orig.z,quat_orig.w] 
        (roll,pitch,yaw)=euler_from_quaternion(quat_list)
        self.pth=yaw

    def run(self):
        path_plan = [[15.3, 0.188, 0.22, 0, 0, 1.5709], 
                [15.3, 0.22, 0.188, 2, 0, -1.5709], 
                [9.1, 0.22, 0.22, 4, 0, 1.5709], 
                [26.5, 0.15, 0.15, 4, 2, -3.14159], 
                [ 9.1, 0.22, 0.22, 0, 2, -1.5709] ]
        Lab2().go_to(path_plan)
        rospy.spin()


if __name__ == '__main__':
    Lab2().run()