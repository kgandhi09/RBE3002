#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point, Twist
from laser_geometry import LaserProjection
import math
from math import sin, cos, pi


class Explore:

    def __init__(self):
        """
        Class constructor
        """
        rospy.init_node('explore', anonymous=True)
        self.laser = LaserScan()
        self.laser_proj = LaserProjection()
        self.pc_pub = rospy.Publisher('/laserPointCloud', PointCloud2, queue_size=1)
        self.sacn_subscriber = rospy.Subscriber('/scan', LaserScan, self.callback)
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        


    def callback(self, msg):
        ranges = list(msg.ranges)
        cloud_out = self.laser_proj.projectLaser(msg)

        front_thresh = 0.27

        right_dist_average = 0
        left_dist_average = 0

        front_dist = 100

        if len(ranges) != 0:
            right_dist = list(ranges[45:134])
            left_dist = list(ranges[225:314])

            if ranges[0] < 100000:
                front_dist = ranges[0]

            right_dist_average = sum(right_dist)/len(right_dist)
            left_dist_average = sum(left_dist)/len(left_dist)

        diff = abs(right_dist_average - left_dist_average)
        
        right = False
        left = False

        if right_dist_average > left_dist_average and diff > 1:
            left = True

        if left_dist_average > right_dist_average and diff > 1:
            right = True

        if front_dist>front_thresh:
            print 'drive straight'
            self.send_speed(0.1,0)
        else:
            if left and (not right):
                print 'turn left'
                self.rotate(pi/2, 0.1)
            if right and (not left):
                print 'turn right'
                self.rotate(pi/2, -0.1)
            if (not right) and (not left):
                print 'drive back'

                
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
            self.send_speed(0,aspeed)
            time = rospy.Time.now().to_sec()
            if aspeed > 0:
                current_angle = aspeed*(time - init_time)
            else:
                current_angle = -aspeed*(time - init_time)
        self.msg_cmd_vel.angular.z = 0
        self.velocity_publisher.publish(self.msg_cmd_vel)

    def run(self):
        self.callback(LaserScan())
        rospy.spin()

if __name__ == "__main__":
    
    explore = Explore()
    explore.run()