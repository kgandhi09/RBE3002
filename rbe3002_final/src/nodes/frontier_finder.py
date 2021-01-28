#!/usr/bin/env python
import math
import rospy
from path_planner import PathPlanner
from rbe3002_lab4.srv import GetPoint
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseWithCovarianceStamped

class FrontierFinder:
	
    def __init__(self):
	## Initialize the node and call it "frontier_finder"
	rospy.init_node("frontier_finder")
	## Create a new service called "find_frontier" that accepts messages of
	##map type and calls self.find_frontier() when a message is received
	self.find_frontier = rospy.Service('find_frontier', GetPoint, self.find_frontier) #hack because we don't use GetMap

	self.mapfrontier = rospy.Publisher('/find_frontier/mapfrontier', GridCells, queue_size = 10)
	self.frontiercentroids = rospy.Publisher('/find_frontier/frontiercentroids', GridCells, queue_size = 10)
        self.cspace = rospy.Publisher('/path_planner/cspace', GridCells, queue_size = 10)


        ## Sleep to allow roscore to do some housekeeping
        rospy.sleep(1.0)
        rospy.loginfo("Frontier finder node ready")



    def calc_centroid(self, F):
	"""
	Calculates the centroid of a region given cell of a region
	:param F [(int, int)] list of cells in frontier region
	:return int, int)] cell of a frontier region

	"""
#	cx = 0
#	cy = 0
#	n = len(F)
#	for cells in F:
#		cx = cx + cells[0]
#		cy = cy + cells[1]
	#for points in F:
#	cx = cx/n
#	cy = cy/n
#	return (cx, cy)
	return F[len(F)/2]

    def find_frontier_points(self, mapdata, robotPos):
	"""
	Calculates the centroid of a cell among the cells of a specific frontier region
	:param mapdata [OccupancyGrid] map
	:return [(int, int)] cell of a frontier region

	"""
	frontier_points = []
	p = 0

	toSearch =[]
	print robotPos
	toSearch.append(robotPos)
	searched = []

	while len(toSearch) != 0:
		(x, y) = toSearch.pop()
		searched.append((x,y))
		
		for point in PathPlanner.neighbors_of_8(mapdata, x, y, 'walkable', []):
			if point not in searched:
				toSearch.append(point)

		if len(PathPlanner.neighbors_of_8(mapdata, x, y, 'unknown', [])) is not 0:
			if (x,y) not in frontier_points:
				frontier_points.append((x,y))

	self.mapfrontier.publish(PathPlanner.points_to_gridcells_message(mapdata, frontier_points))
	print "number frontier points: " + str(len(frontier_points))
	return frontier_points

    def find_closest_frontier_point(self, mapdata, robotPos):
	"""
	Calculates the centroid of a cell among the cells of a specific frontier region
	:param mapdata [OccupancyGrid] map
	:return [(int, int)] cell of a frontier region

	"""
	frontier_points = []
	p = 0

	toSearch =[]
	print robotPos
	toSearch.append(robotPos)
	searched = []

	while len(toSearch) != 0:
		(x, y) = toSearch.pop()
		searched.append((x,y))
		
		for point in PathPlanner.neighbors_of_8(mapdata, x, y, 'walkable', []):
			if point not in searched:
				toSearch.append(point)

		if len(PathPlanner.neighbors_of_8(mapdata, x, y, 'unknown', [])) is not 0:
			region = self.add_frontier_points(mapdata, (x,y))
			
			if(len(region) > 0):
				return region
			else:
				for r in region:
					searched.append(r)

	#TODO move self.mapfrontier.publish(PathPlanner.points_to_gridcells_message(mapdata, frontier_points))
	return None


    def add_frontier_points(self, mapdata, frontier_point):
	region = []
	to_search = []

	to_search.append(frontier_point)

	while len(to_search) > 0:

		point = to_search.pop()

		region.append(point)

		for p in PathPlanner.neighbors_of_8(mapdata, point[0], point[1], 'walkable', []):
			if p not in region and p not in to_search and  (len(PathPlanner.neighbors_of_8(mapdata, p[0], p[1], 'unknown', [])) > 0):
				to_search.append(p)


	self.mapfrontier.publish(PathPlanner.points_to_gridcells_message(mapdata, region))
	
	print region
	return region


			
    def combine_frontier_points(self, mapdata, frontier_points):
	regions = []

	to_search = frontier_points

	while len(to_search) > 0:
		cur_region = []
		point = to_search[0]
		
		region_to_search = []
		region_to_search.append(point)
		to_search.remove(point)

		while len(region_to_search) > 0:
			point = region_to_search[0]
			cur_region.append(point)
			region_to_search.remove(point)
			for neighbor in PathPlanner.neighbors_of_8(mapdata, point[0], point[1], 'input', frontier_points):
				region_to_search.append(neighbor)
				to_search.remove(neighbor)

		print "Region: " + str(len(cur_region))
		regions.append(cur_region)

			

	return regions



    def find_frontier(self, msg):
	mapdata = PathPlanner.request_map()
	self.frontiercentroids.publish(PathPlanner.points_to_gridcells_message(mapdata, []))
	self.mapfrontier.publish(PathPlanner.points_to_gridcells_message(mapdata, []))

	robotPose = PathPlanner.world_to_grid(mapdata, msg.robotPos)

        (mapdata.data, added) = PathPlanner.calc_cspace(mapdata, 1)
	self.cspace.publish(PathPlanner.points_to_gridcells_message(mapdata, added))

	region = self.find_closest_frontier_point(mapdata, robotPose)
	c = self.calc_centroid(region)
	c = PathPlanner.grid_to_world(mapdata, c[0], c[1])

	p = Point()
	p.x = c[0]
	p.y = c[1]
	
	#regions = self.add_frontier_points(mapdata, frontier_point)
	#print regions
	self.frontiercentroids.publish(PathPlanner.points_to_gridcells_message(mapdata, [c]))
	#p = self.choose_region(region, robotPose, mapdata)
	print "RESPONSE: " 
	print p
	return p


    def choose_region(self, regions, robotPos, mapdata):
	#regions = self.filter_regions(regions, 1)
	
	if len(regions) == 0:
		return Point()

	centroids = []
	shortestDistance = mapdata.info.width + mapdata.info.height 
	shortestCentroid = (0,0)
	for r in regions:
		c = self.calc_centroid(r)
		centroids.append(c)
		d = PathPlanner.euclidean_distance(c[0], c[1], robotPos[0], robotPos[1])
		print "distance " + str(d)
		if d < shortestDistance:
			shortestCentroid = c
			shortestDistance = d
	
	p = Point()
			
	print "Centroid chosen: "
	print shortestCentroid
	(x, y) = PathPlanner.grid_to_world(mapdata, shortestCentroid[0], shortestCentroid[1])
	p.x = x
	p.y = y

	self.frontiercentroids.publish(PathPlanner.points_to_gridcells_message(mapdata, centroids))
	
	return p


    def filter_regions(self, regions, size):
	new_regions = []
	for r in regions:
		if len(r) >= size:
			new_regions.append(r)
	return new_regions

    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        rospy.spin()

if __name__ == '__main__':
    FrontierFinder().run()
