#!/usr/bin/env python
import math
import numpy
import rospy
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from std_msgs.msg import String
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseWithCovarianceStamped
import priority_queue as pq

class PathPlanner:
    def __init__(self):
        """
        Class constructor
        """
        ### REQUIRED CREDIT
        ## Initialize the node and call it "path_planner"
        rospy.init_node("path_planner")
        ## Create a new service called "plan_path" that accepts messages of
        ## type GetPlan and calls self.plan_path() when a message is received
        self.plan_path = rospy.Service('plan_path', GetPlan, self.plan_path)
        ## Create publishers for the C-space (the enlarged occupancy grid)
        ## The topic is "/path_planner/cspace", the message type is GridCells
        self.cspace = rospy.Publisher('/path_planner/cspace', GridCells, queue_size = 10)

        ## Create publishers for A* (expanded cells, frontier, ...)
        ## Choose a the topic names, the message type is GridCells
        self.showpathGC = rospy.Publisher('/path_planner/showpathGC', GridCells, queue_size = 10)
	self.showvisited = rospy.Publisher('/path_planner/showvisited', GridCells, queue_size = 10)
	self.showwaypoints = rospy.Publisher('/path_planner/showwaypoints', GridCells, queue_size = 10)
	self.showfrontier = rospy.Publisher('/path_planner/showfrontier', GridCells, queue_size = 10)
	self.showpathP = rospy.Publisher('/path_planner/showpathP', Path, queue_size = 10)
	rospy.Subscriber('/state', String,  self.state_update) 
	self.path_amount = 1

        ## Sleep to allow roscore to do some housekeeping
        rospy.sleep(1.0)
        rospy.loginfo("Path planner node ready")


    def state_update(self, msg):
	if msg.data == 'MAPPING':
		self.path_amount = .75
	else:
		self.path_amount = 1

    @staticmethod
    def grid_to_index(mapdata, x, y):
        """
        Returns the index corresponding to the given (x,y) coordinates in the occupancy grid.
        :param x [int] The cell X coordinate.
        :param y [int] The cell Y coordinate.
        :return  [int] The index.
        """
  	return y*mapdata.info.width + x

    @staticmethod
    def index_to_grid(mapdata, i):
	width = mapdata.info.width
	y = i/width
	x = i - width*y	
	return (x,y)

    @staticmethod
    def euclidean_distance(x1, y1, x2, y2):
        """
        Calculates the Euclidean distance between two points.
        :param x1 [int or float] X coordinate of first point.
        :param y1 [int or float] Y coordinate of first point.
        :param x2 [int or float] X coordinate of second point.
        :param y2 [int or float] Y coordinate of second point.
        :return   [float]        The distance.
        """
	x_dif = x1-x2
	y_dif = y1-y2
	return math.sqrt((x_dif**2.0)+(y_dif**2.0))
        
    @staticmethod
    def grid_to_world(mapdata, x, y):
        """
        Transforms a cell coordinate in the occupancy grid into a world coordinate.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The cell X coordinate.
        :param y       [int]           The cell Y coordinate.
        :return        [Point]         The position in the world.
        """
	wc_x = (x + 0.5) * mapdata.info.resolution + mapdata.info.origin.position.x
	wc_y = (y + 0.5) * mapdata.info.resolution + mapdata.info.origin.position.y
		
	return (wc_x, wc_y)
        ### REQUIRED CREDIT
	
    @staticmethod
    def world_to_grid(mapdata, wp):
        """
        Transforms a world coordinate into a cell coordinate in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param wp      [Point]         The world coordinate.
        :return        [(int,int)]     The cell position as a tuple.
        """
        ### REQUIRED CREDIT
	gc_x = int((wp.x - mapdata.info.origin.position.x) / mapdata.info.resolution)
	gc_y = int((wp.y - mapdata.info.origin.position.y) / mapdata.info.resolution)
        return (gc_x,gc_y)


    @staticmethod
    def points_to_gridcells_message(mapdata, points):
        """
        Converts the given points into a GridCells message.
        :param mapdata [OccupancyGrid] The map information.
        :param  path   [[(int,int)]]   The points as a list of tuples (cell coordinates).
        :return        [Gridcells] The points as a GridCells message(world coordinates).
        """
	#rospy.loginfo("Creating a Path message")

	msg = GridCells()
		
	msg.cell_width = mapdata.info.resolution
	msg.cell_height = mapdata.info.resolution
	msg.header.frame_id = 'map'

	cells = []
	for point in points:
		p = Point()
		world_cord = PathPlanner.grid_to_world(mapdata, point[0], point[1])
		p.x = world_cord[0]
		p.y = world_cord[1]
		p.z = 0				
		cells.append(p)

	msg.cells = cells
	
	return msg

    @staticmethod
    def is_cell(mapdata, x, y, param, points):
        """
        A cell is walkable if all of these conditions are true:
        1. It is within the boundaries of the grid;
        2. It is free (not unknown, not occupied by an obstacle)
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [boolean]       True if the cell is walkable, False otherwise
        """
        ### REQUIRED CREDIT

	if(param == 'walkable'):
		if x < mapdata.info.width and x >= 0 and y < mapdata.info.height and y >= 0:
			i = PathPlanner.grid_to_index(mapdata, x, y)
			if mapdata.data[i] < 50 and mapdata.data[i] is not -1:
				return True 

	if(param == 'unknown'):
		if x < mapdata.info.width and x >= 0 and y < mapdata.info.height and y >= 0:
			i = PathPlanner.grid_to_index(mapdata, x, y)
			if mapdata.data[i] is -1:
				return True 

	if(param == 'input'):
		if x < mapdata.info.width and x >= 0 and y < mapdata.info.height and y >= 0:
			if (x,y) in points:
				return True 
	#condition
	return False          

    @staticmethod
    def neighbors_of_4(mapdata, x, y, param, frontier_points):
        """
        Returns the walkable 4-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 4-neighbors.
        """

	arr = []
	
	if PathPlanner.is_cell(mapdata, x+1, y, param, frontier_points):
		arr.append((x+1, y))
	if PathPlanner.is_cell(mapdata, x, y+1, param, frontier_points):
		arr.append((x, y+1))
	if PathPlanner.is_cell(mapdata, x-1, y, param, frontier_points):
		arr.append((x-1, y))
	if PathPlanner.is_cell(mapdata, x, y-1, param, frontier_points):
		arr.append((x, y-1))
	return arr

    @staticmethod
    def neighbors_of_8(mapdata, x, y, param, frontier_points):
        """
        Returns the walkable 8-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 8-neighbors.
        """
	arr = PathPlanner.neighbors_of_4(mapdata, x, y, param, frontier_points)

	if PathPlanner.is_cell(mapdata, x+1, y+1, param, frontier_points):
		arr.append((x+1, y+1))
	if PathPlanner.is_cell(mapdata, x+1, y-1, param, frontier_points):
		arr.append((x+1, y-1))
	if PathPlanner.is_cell(mapdata, x-1, y-1, param, frontier_points):
		arr.append((x-1, y-1))
	if PathPlanner.is_cell(mapdata, x-1, y+1, param, frontier_points):
		arr.append((x-1, y+1))
	return arr

    @staticmethod
    def request_map():
        """
        Requests the map from the map server.
        :return [OccupancyGrid] The grid if the service call was successful,
                                None in case of error.
        """
        ### REQUIRED CREDIT
	#rospy.loginfo("waiting for service to request map")
	rospy.wait_for_service('/dynamic_map')
	try:
		rospy.loginfo("Requesting the map")
		sp = rospy.ServiceProxy('/dynamic_map', GetMap)
		m = sp().map
		rospy.loginfo("Map requset complete")
		return m
	except:
		rospy.loginfo("Unable to request map")



    @staticmethod	
    def calc_cspace(mapdata, padding):
        """
        Calculates the C-Space, i.e., makes the obstacles in the map thicker.
        Publishes the list of cells that were added to the original map.
        :param mapdata [OccupancyGrid] The map data.
        :param padding [int]           The number of cells around the obstacles.
        :return        [OccupancyGrid]        The C-Space as a new map.
        """
        ### REQUIRED CREDIT
        rospy.loginfo("Calculating C-Space with padding " + str(padding))

	map_orig = list(mapdata.data)
	map_final = list(mapdata.data)

	added = []
	width = mapdata.info.width
	height = mapdata.info.height

	## Go through each cell in the occupancy grid
	## Inflate the obstacles where necessary
	i = int(0)
	while i < len(map_orig):
		if map_orig[i] > 50:
			#calcualte x and y location of the current point
			y_orig = i/width
			x_orig = i - width*y_orig

			#iterate through all x points between (x_orig - padding)  or (zero) whichever is higher
			#and (x_orig + padding) or (map_data.width - 1) whicher is lower (inclusive)
			x = x_orig - padding 
			while x <= x_orig + padding and x < width:
				if x >= 0:
					#iterate through all y points between (y_orig - padding)  or (zero) whichever is higher
					#and (y_orig + padding) or (map_data.height - 1) whicher is lower (inclusive)
					y = y_orig - padding 
					while y <= y_orig + padding and y < height:
						if y >= 0:
							#calculate current index from (x, y) and update value in final map
							j = PathPlanner.grid_to_index(mapdata, x, y)
							map_final[j] = map_orig[i]
							if (x,y) not in added:
								added.append((x,y))
						y = y + 1	
				x = x + 1
		i = i + 1  

	rospy.loginfo("algorithm complete")
	
	## Create a GridCells message and publish it 
	#self.cspace.publish(PathPlanner.points_to_gridcells_message(mapdata, added))

        ## Return the C-space
	return (map_final, added)

    def a_star(self, mapdata, cspace, start, goal):
        ### REQUIRED CREDIT

	rospy.loginfo("Executing A* from (%d,%d) to (%d,%d)" % (start[0], start[1], goal[0], goal[1]))
	
	#remove previous search from visualzation
	self.showpathGC.publish(PathPlanner.points_to_gridcells_message(mapdata, []))
	self.showvisited.publish(PathPlanner.points_to_gridcells_message(mapdata, []))
	self.showfrontier.publish(PathPlanner.points_to_gridcells_message(mapdata, []))
	self.showwaypoints.publish(PathPlanner.points_to_gridcells_message(mapdata, []))
	self.path_to_message(mapdata, [])

	#A* variables
	frontier = pq.PriorityQueue()
	frontier.put(start, PathPlanner.euclidean_distance(start[0], start[1], goal[0], goal[1]))
	frontierPub = [] #for visualization
	frontierPub.append(start)
	came_from = {}
	g_n = {}
	f_n = {}
	came_from = {}
	came_from[start] = None
	g_n[start] = 0
	path = []
	visited = []
	cell_fn = {}
	turn_penalty = {}

	current = start
	came_from[current] = start

	while not frontier.empty():
		current = frontier.get() #get first node out of priority que
		frontierPub.remove(current) # remove from frontier visualizaiton
		
		if not current in visited:	
			visited.append(current) #add to list of visited nodes
			self.showvisited.publish(PathPlanner.points_to_gridcells_message(mapdata, visited))

                #if found the goal get the path by walking through visited nodes backwards in order of
		#where they came from until the start is reached.
		if current == goal:			
			while current is not start:
				path.append(current)
				current = came_from[current]
			path.append(start)
			path.reverse()
			break

		#iterate through current's negibors, calcualte their f(n) and and them to the priority que if they aren't in it yet
		#or a lower f(n) has been found
		for next in PathPlanner.neighbors_of_8(mapdata, current[0], current[1], 'walkable', []):
				if next not in visited:

					turn_penalty = 0
					if not PathPlanner.along_same_path(came_from[current], current, next):
						turn_penalty  = turn_penalty + .7

					c_space_penalty = 0
					if not len(PathPlanner.neighbors_of_8(mapdata, next[0], next[1], 'input', cspace)) == 0:
						c_space_penalty = c_space_penalty + 2

					new_g_n = g_n[current] + PathPlanner.euclidean_distance(current[0], current[1], next[0], next[1]) + turn_penalty + c_space_penalty

					if (next not in g_n) or (new_g_n < g_n[next]):
						came_from[next] = current
						g_n[next] = new_g_n
						f_n = g_n[next] + PathPlanner.euclidean_distance(next[0], next[1], goal[0], goal[1])
						frontier.put(next,f_n)	
						cell_fn[next] = f_n

					if next not in frontierPub:
						frontierPub.append(next)
		
		self.showfrontier.publish(PathPlanner.points_to_gridcells_message(mapdata, frontierPub))	
		
	#print cell_fn

	#publish the final path
	self.showpathGC.publish(PathPlanner.points_to_gridcells_message(mapdata, path))

	return path
	
    @staticmethod
    def along_same_path(prv, cur, next):

	if prv == cur or cur == next:
		return True

	mag_a = PathPlanner.euclidean_distance(prv[0], prv[1], cur[0], cur[1]);
	mag_b = PathPlanner.euclidean_distance(cur[0], cur[1], next[0], next[1]);

	dot_product = (cur[0]-prv[0])*(next[0]-cur[0])+ (cur[1]-prv[1])*(next[1]-cur[1]);

	cos_theta = dot_product/(mag_a*mag_b);

	return 	cos_theta > .999; #cos_theta = 1

    @staticmethod
    def optimize_path(path):
        """
        Optimizes the path, removing unnecessary intermediate nodes.
        :param path [[(x,y)]] The path as a list of tuples (grid coordinates)
        :return     [[(x,y)]] The optimized path as a list of tuples (grid coordinates)
        """
        ### EXTRA CREDIT
        rospy.loginfo("Optimizing path")
	toRemove = []
	i = 1

	#stage nodes for removal if they are along the same horizontal, vertical, or diagonal as both the previous and the next node
	while i < len(path) - 1:

		prv = path[i-1] 
		cur = path[i]
		next = path[i+1]

		if PathPlanner.along_same_path(prv, cur, next):
			toRemove.append(i)

		i = i + 1
	
	
	#remove nodes stages for removal
	sub = 0 #keep track of number of removed nodes to adjust index accordingly
	for index in toRemove:
		path.pop(index - sub)
		sub = sub + 1

	return path

    @staticmethod
    def optimize_path_vector(mapdata, path):
	
	op_path = []
	i = 0
	while i < len(path) - 1:
		cur_point = path[i]
		op_path.append(cur_point)
		j = i + 1
		while PathPlanner.can_traverse_vector(mapdata, cur_point, path[j]):
			if j == len(path) - 1:
				op_path.append(path[len(path)-1])
				break;
			j = j + 1
		if i == j:
			i = i + 1
		else:
			i = j


	print "VECTOR OPTIMIZED PATH: "
	print op_path
	return op_path
		
	

    @staticmethod
    def can_traverse_vector(mapdata, A, B):
    #Return all cells of the unit grid crossed by the line segment between
     #   A and B.
#https://stackoverflow.com/questions/35807686/find-cells-in-array-that-are-crossed-by-a-given-line-segment
    #

	(xA, yA) = A
	(xB, yB) = B
    	(dx, dy) = (xB - xA, yB - yA)
    	(sx, sy) = (numpy.sign(dx), numpy.sign(dy))

    	grid_A = (math.floor(A[0]), math.floor(A[1]))
    	grid_B = (math.floor(B[0]), math.floor(B[1]))
    	(x, y) = grid_A

    	tIx = dy * (x + sx - xA) if dx != 0 else float("+inf")
    	tIy = dx * (y + sy - yA) if dy != 0 else float("+inf")

    	while (x,y) != grid_B:
		# NB if tIx == tIy we increment both x and y
		(movx, movy) = (tIx <= tIy, tIy <= tIx)

		if movx:
		    # intersection is at (x + sx, yA + tIx / dx^2)
		    x += sx
		    tIx = dy * (x + sx - xA)

		if movy:
		    # intersection is at (xA + tIy / dy^2, y + sy)
		    y += sy
		    tIy = dx * (y + sy - yA)
		
		if not PathPlanner.is_cell(mapdata, int(x), int(y), 'walkable', []):
			return False
    	return True


    def path_to_message(self, mapdata, path):
        """
        Takes a path on the grid and returns a Path message.
        :param path [[(int,int)]] The path on the grid (a list of tuples)
        :return     [Path]        A Path message (the coordinates are expressed in the world)
        """
        ### REQUIRED CREDIT
	msg = Path()
	
	msg.header.frame_id = 'map' 
	
	for point in path:
		msg_pose = PoseStamped()
		world_cord = PathPlanner.grid_to_world(mapdata, point[0], point[1])

		msg_pose.pose.position.x = world_cord[0]
		msg_pose.pose.position.y = world_cord[1]
		msg_pose.pose.position.z = 0

		msg.poses.append(msg_pose)

	self.showpathP.publish(msg)
	return msg

    @staticmethod
    def reduce_path(path, percent):
	amount = int(len(path)*percent)
	print "REDUCE_______________"
	print len(path)
	print amount
	print path

	i = len(path) - 1
	while i >= amount and i >= 0:
		path.remove(path[i])
		i = i - 1

	print path
	return path


    def plan_path(self, msg):
        """
        Plans a path between the start and goal locations in the requested.
        Internally uses A* to plan the optimal path.
        :param req 
        """
        ## Request the map
        ## In case of error, return an empty path
        mapdata = PathPlanner.request_map()
        if mapdata is None:
            return Path()

        ## Calculate the C-space and publish it
        (mapdata.data, cspace) = PathPlanner.calc_cspace(mapdata, 1)
	self.cspace.publish(PathPlanner.points_to_gridcells_message(mapdata, cspace))

        ## Execute A*
        start = PathPlanner.world_to_grid(mapdata, msg.start.pose.position)
        goal  = PathPlanner.world_to_grid(mapdata, msg.goal.pose.position)
        path  = self.a_star(mapdata, cspace, start, goal)

        ## Optimize waypoints
	reduced_path = PathPlanner.reduce_path(path, self.path_amount)
        waypoints = PathPlanner.optimize_path_vector(mapdata, path) #PathPlanner.optimize_path(reduced_path)
	self.showwaypoints.publish(PathPlanner.points_to_gridcells_message(mapdata, path))
        ## Return a Path message
        return self.path_to_message(mapdata, waypoints)


    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        rospy.spin()

if __name__ == '__main__':
    PathPlanner().run()
