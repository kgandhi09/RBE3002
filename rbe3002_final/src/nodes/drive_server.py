#!/usr/bin/env python
import math
import rospy
import tf
from nav_msgs.srv import GetPlan, GetMap
from rbe3002_lab4.srv import SendPlan
from nav_msgs.msg import GridCells, OccupancyGrid, Path, Odometry
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseWithCovarianceStamped, Twist
from tf.transformations import euler_from_quaternion


#TODO add timeout for driving - doesn't get stuck trying to drive into a wall
#TODO add way to get out of c-space if gets stuck in c-space - could be done in pathplanner by just moving start
#TODO after, decrease c-space in frontier_finder to 2. Could increase path_planner c-space and make it drive all the way to get the robot stuck
#TODO trim beigining of path instead of skipping first waypoint??
#TODO doesn't end search when no more waypoints + somehow still finding places to try and get to
class DriveServer:
	def __init__(self):
		## Initialize the node and call it  "path_planner"
        	rospy.init_node("drive_server")

        	## Create a new service called "plan_path" that accepts messages of
        	## type GetPlan and calls self.plan_path() when a message is received
        	s = rospy.Service('drive_path', SendPlan, self.drive_path)

		#subscriber nodes for getting start and goal

        	#subcriber to 2D pose estimate from rviz to identify a start position without a moving robot
		#rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.update_cur_position)
        	

		self.curPos = PoseStamped()

		### Tell ROS that this node publishes Twist messages on the '/cmd_vel' topic
        	self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		
        	
        	#this node subscribes to Odometry messages on the '/odom' topic
        	#to update the robots position for the start and for robot driving
       		rospy.Subscriber('/main/robot_position', PoseStamped, self.update_position)
        	self.px = 0
        	self.py = 0
        	self.pth = 0

		rospy.sleep(1.0)


		self.listener = tf.TransformListener()
		self.listener.waitForTransform('/map', '/base_footprint', rospy.Time(0), rospy.Duration(4.0))

        	## Sleep to allow roscore to do some housekeeping
        	rospy.sleep(1.0)
        	rospy.loginfo("Path client node ready")


	def drive_path(self, path):
                """
		Calls go_to() for all waypoints in path to traverse a given path.
		:param msg [Path] The path to traverse
		"""
		rospy.loginfo("In drive_path")

		first_pose = True
		for point in path.plan.poses:
			print point
			if not first_pose:
				self.go_to(point)
			else:
				first_pose = False

		rospy.loginfo("drive_path complete")
		return "complete"


	def run(self):
        	"""
        	Runs the node until Ctrl-C is pressed.
        	"""
        	rospy.spin()

	def go_to(self, msg):
		"""
		Calls rotate() and drive() to attain a given position.
		:param msg [PoseStamped] The target pose.
		"""
		PI = math.pi

		### REQUIRED CREDIT

		#determine x and y distance components
		delta_x = msg.pose.position.x - self.px
		delta_y = msg.pose.position.y - self.py 

		#determines angle between current pose and distance vector between [-PI and PI]
		theta = math.fmod(math.fmod((math.atan2(delta_y, delta_x) + 2.0*PI),(2.0*PI)) - math.fmod((self.pth + 2.0*PI),(2.0*PI)) + PI, 2.0*PI) - PI

		#calculates distance vector
		distance = math.sqrt(delta_x**2.0 + delta_y**2.0)

		#rotate to face the final position
		self.rotate(theta, 0.2)

		#drive to the final position
		self.drive(distance,0.1)

	def drive(self, distance, linear_speed):
		"""
		Drives the robot in a straight line by setting specified linear speed
		then moving till the distance moved is within a tolerance of the
		requested distance
		:param distance     [float] [m]   The distance to cover.
		:param linear_speed [float] [m/s] The forward linear speed.
		"""
		rospy.loginfo('drive ' + str(distance) + ' m at ' + str(linear_speed) + ' m/s')

		### REQUIRED CREDIT
		xi = self.px
		yi = self.py
		tolerance = .01

		x_traveled = 0.0
		y_traveled = 0.0

		self.send_speed([linear_speed,0.0,0.0], [0.0,0.0,0.0])

		while math.sqrt((x_traveled**2.0)+(y_traveled**2.0)) <= (distance-tolerance):
		        x_traveled = self.px - xi
		        y_traveled = self.py - yi
		        rospy.sleep(.05)

		self.send_speed([0.0,0.0,0.0], [0.0,0.0,0.0])

	def rotate(self, angle, aspeed):
		"""
		Rotates the robot around the body center by the given angle at the given speed.
		Chooses directon to rotate based on shortest turn
		:param angle         [float] [rad]   The distance to cover.
		:param angular_speed [float] [rad/s] The angular speed.
		"""
		rospy.loginfo('rotate ' + str(angle) +  ' rad at ' + str(aspeed) + ' rad/s')

		PI = math.pi
		tolerance = .05 #rad

		#make angle between [0,2pi]
		while angle < 0.0:
		        angle = 2.0*PI + angle
		angle = math.fmod(angle,2.0*PI)
		
		#inital position between [0, 2PI]:
		init_pos = math.fmod(self.pth + 2.0*PI, 2.0*PI)
		
		#calculate final position between [0, 4PI] using angle and current position
		final_pos = angle + init_pos
		
		#move final position to [-PI, PI]
		final_pos = math.fmod(final_pos + PI,2.0*PI) - PI

		#choose direciton to rotate based off the angle
		aspeed = abs(aspeed)
		if angle > PI:
		        aspeed = -aspeed

		#send angular speed
		self.send_speed([0.0, 0.0, 0.0],[0.0, 0.0, aspeed])

		#wait for robot to be in the tolerance
		while abs(self.pth-final_pos) > tolerance:
		        rospy.sleep(.05)

		        
		self.send_speed([0.0, 0.0, 0.0],[0.0, 0.0, 0.0])

	def send_speed(self, linear_speed, angular_speed):
		"""
		Sends the speeds to the motor by publishing to /cmd_vel topic 
		:param linear_speed  {v_x, v_y, v_z} [m/s]   The forward linear speed.
		:param angular_speed {w_x, w_y, w_z} [rad/s] The angular speed for rotating around the body center.
		"""
		### REQUIRED CREDIT
		### Make a new Twist message
		# Create twist message
		msg_cmd_vel = Twist()
		# Linear velocity
		msg_cmd_vel.linear.x = linear_speed[0]
		msg_cmd_vel.linear.y = linear_speed[1]
		msg_cmd_vel.linear.z = linear_speed[2]
		# Angular velocity
		msg_cmd_vel.angular.x = angular_speed[0]
		msg_cmd_vel.angular.y = angular_speed[1]
		msg_cmd_vel.angular.z = angular_speed[2]
		### Publish the message
		# Send command
		self.cmd_vel.publish(msg_cmd_vel)

	def update_position(self, msg):
		self.px = msg.pose.position.x
		self.py = msg.pose.position.y

       		quat_orig = msg.pose.orientation
       		quat_list = [quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]

		#convert quaternion to euler angles
		(roll, pitch, yaw) = euler_from_quaternion(quat_list)

		#set orientaiton
		self.pth = yaw




if __name__ == '__main__':
	DriveServer().run()
