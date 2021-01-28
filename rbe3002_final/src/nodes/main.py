#!/usr/bin/env python
import math
import rospy
import tf
import copy
from nav_msgs.srv import GetPlan, GetMap
from rbe3002_lab4.srv import GetPoint, SendPlan
from std_msgs.msg import String
from nav_msgs.msg import GridCells, OccupancyGrid, Path, Odometry
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseWithCovarianceStamped, Twist, PointStamped
from tf.transformations import euler_from_quaternion

class MainClient:
	def __init__(self):
		## Initialize the node and call it  "path_planner"
        	rospy.init_node("main_client")

		#subscriber nodes for getting start and goal

        	#subcriber to 2D pose estimate from rviz to identify a start position without a moving robot
		#rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.update_cur_position)
        	
		#rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.request_path)
		self.curPos = PoseStamped()

		rospy.sleep(1.0)

		self.listener = tf.TransformListener()
		self.listener.waitForTransform('/map', '/base_footprint', rospy.Time(0), rospy.Duration(4.0))
		
		self.publish_pos = rospy.Publisher('/main/robot_position', PoseStamped, queue_size=10)
		self.publish_start_pos = rospy.Publisher('/main/start_position', PointStamped, queue_size=10)
		self.publish_state = rospy.Publisher('/state', String, queue_size=10)
		

		#this node subscribes to Odometry messages on the '/odom' topic
        	#to update the robots position for the start and for robot driving
       		rospy.Subscriber('/odom', Odometry, self.update_odometry) 
		rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.to_goal)

        	## Sleep to allow roscore to do some housekeeping
        	rospy.sleep(1.0)
        	rospy.loginfo("Main node ready")

	def update_odometry(self, msg):
		"""
		Updates the current pose of the robot by setting px, py and pth whenever
		/Odom publishes
		This method is a callback bound to the /odom subscriber.
		:param msg [Odometry] The current odometry information.
		"""

		try:
			(trans, rot) = self.listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))

			self.curPos.pose.position.x = trans[0]
			self.curPos.pose.position.y = trans[1]
			self.curPos.pose.position.z = trans[2]
			self.curPos.pose.orientation.x = rot[0]
			self.curPos.pose.orientation.y = rot[1]
			self.curPos.pose.orientation.z = rot[2]
			self.curPos.pose.orientation.w = rot[3]
			self.curPos.header = msg.header
			
			self.publish_pos.publish(self.curPos)

		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			print "ERROR"


	def request_frontier_point(self):
		rospy.loginfo("start request frontier point function")
		
                #wait for service node
		rospy.wait_for_service('find_frontier')

		msgGetPoint = GetPoint()
		msgGetPoint.robotPos = self.curPos.pose.position
		
		#print msgGetPoint
		try:
			rospy.loginfo("Requesting point")
			#create service proxy, send request with paramaters and store response
			sp = rospy.ServiceProxy('find_frontier', GetPoint)
			#print sp
			pointg = sp(msgGetPoint.robotPos)
			
			path_goal = PoseStamped()
			path_goal.pose.position.x = pointg.point.x
			path_goal.pose.position.y = pointg.point.y
			
		
			return path_goal
		except:
			rospy.loginfo("Unable to request point")
			return None

		rospy.loginfo("Request_frontier_point function complete")

	def request_path(self, goal):
		rospy.loginfo("start request path function")
		
                #create get plan message
		msgGetPlan = GetPlan()
		msgGetPlan.start = self.curPos
		msgGetPlan.goal = goal
		msgGetPlan.tolerance = .5
		path = Path()

		rospy.loginfo(msgGetPlan.start)
		rospy.loginfo(msgGetPlan.goal)

                #wait for service node
		rospy.wait_for_service('plan_path')
		try:
			rospy.loginfo("Requesting path")

			#create service proxy, send request with paramaters and store response
			sp = rospy.ServiceProxy('plan_path', GetPlan)
			path = sp(msgGetPlan.start, msgGetPlan.goal, msgGetPlan.tolerance)
			#print path
			return path.plan
		except:
			rospy.loginfo("Unable to request path")
			return None


		rospy.loginfo("Request_path function complete")

  	def request_drive(self, path):

                #wait for service node
		rospy.wait_for_service('drive_path')
		msg = Path()
		msg = path


		#print msg

		#print '++++++++++++++'
		#print msg.plan

		try:
			rospy.loginfo("Requesting drive")

			#create service proxy, send request with paramaters and store response
			sp = rospy.ServiceProxy('drive_path', SendPlan)
			print "SERVCE PROXY CREATED"
			resp = sp(msg)
			print "RESPONSE: " + resp
		except:
			return None
			rospy.loginfo("Unable to request drive")


		rospy.loginfo("request_drive function complete")

	def explore(self):
		goal = self.request_frontier_point()

	def to_goal(self, msg):
		goal = msg
		path = self.request_path(goal)
		resp = self.request_drive(path)

	def run(self):
		"""
		Runs the node until Ctrl-C is pressed.
		"""
		done = 0

		startPos = copy.deepcopy(self.curPos)

		startPub = PointStamped()
		startPub.point = startPos.pose.position
		startPub.header = startPos.header
		self.publish_start_pos.publish(startPub)

		print "startPos_______________________________________________________" 
		print startPos

		#PART 1: MAPPING
		state = String()
		state.data = 'MAPPING'
		self.publish_state.publish(state)
		while done < 6:
			goal = self.request_frontier_point()
			if goal == None:
				done = done + 1
			else:
				path = self.request_path(goal)
				if path == None:
					done = done + 1
				else:
					resp = self.request_drive(path)
					if resp == None:
						pass
						#done = done + 1
					else: 
						done = 0

			print "startPos_______________________________________________________" 
			print startPos
		
		print "MAP COMPLETE"

		#PART 2: GO BACK TO START
		state.data = 'RETURN'
		self.publish_state.publish(state)
		path = self.request_path(startPos)
		resp = self.request_drive(path)
		#TODO drive all the way back (not cut off the path)

		print "BACK AT START"

		#PART 3: GO TO GOAL
		

		rospy.spin()

if __name__ == '__main__':
	MainClient().run()


