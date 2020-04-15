#!/usr/bin/env python2

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from std_msgs.msg import String
from math import sqrt
from math import pi
from math import atan2

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

        # pass


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


    def go_to(self, msg):
        """
        Calls rotate(), drive(), and rotate() to attain a given pose.
        This method is a callback bound to a Subscriber.
        :param msg [PoseStamped] The target pose.
        """
        #REQUIRED CREDIT
        pos_x = msg.pose.position.x
        pos_y = msg.pose.position.y
        pos_z = msg.pose.position.z
        ort_x = msg.pose.orientation.x
        ort_y = msg.pose.orientation.y
        ort_z = msg.pose.orientation.z
        ort_w = msg.pose.orientation.w
        print(pos_x,pos_y,pos_z,ort_x,ort_y,ort_z,ort_z,ort_w)
        Lab2().update_odometry(Odometry())
        angle = atan2((pos_y - self.py), (pos_x - self.py))
        if angle != 0:
            if angle < 0:
                Lab2().rotate(angle, -0.1)
            else:
                Lab2().rotate(angle, 0.1)
        distance = sqrt( ((pos_x - self.px) ** 2) + ((pos_y - self.py) ** 2) )
        if distance != 0:
            Lab2().drive(distance, 0.1)
        new_angle = atan2((ort_y - self.py), (ort_x - self.py))
        # if new_angle != 0:
        #     if new_angle < 0:
        #         Lab2().rotate(new_angle, -0.1)
        #     else:
        #         Lab2().rotate(new_angle, 0.1)
        # self.pose_subscriber.callback

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


    def arc_to(self, position):
        """
        Drives to a given position in an arc.
        :param msg [PoseStamped] The target pose.
        """
        # EXTRA CREDIT
        # TODO
        pass # delete this when you implement your code


    def smooth_drive(self, distance, linear_speed):
        """
        Drives the robot in a straight line by changing the actual speed smoothly.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The maximum forward linear speed.
        """
        # EXTRA CREDIT
        # TODO
        pass # delete this when you implement your code



    def run(self):
        # Lab2().__init__()
        # rospy.spin()
        # Lab2.drive(self, 10, 0.2)
        # Lab2().rotate(2*pi, -0.1)
        Lab2().go_to(PoseStamped())
        rospy.spin()


if __name__ == '__main__':
    Lab2().run()
