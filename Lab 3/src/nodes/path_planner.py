#!/usr/bin/env python

import math
import rospy
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped
from std_msgs.msg import Header
from Queue import PriorityQueue


class PathPlanner:

    def __init__(self):
        """
        Class constructor
        """
        self.path = []
        rospy.init_node("path_planner")
        # self.getplan = rospy.Service("plan_path", GetPlan, self.plan_path)

        self.cspace_pub = rospy.Publisher("/path_planner/cspace", GridCells, queue_size=1)
        self.a = rospy.Service("a_star", GetPlan, self.handle_a_star)
        self.expanded = rospy.Publisher("/path_planner/expanded", GridCells, queue_size=1)
        self.frontier = rospy.Publisher("/path_planner/frontier", GridCells, queue_size=1)
        self.pose_subscriber = rospy.Subscriber('/move_base_simple/goal', PoseStamped)

        # TODO
        # Initialize the request counter
        # TODO
        rospy.sleep(1.0)
        rospy.loginfo("Path planner node ready")

        grid = PathPlanner.request_map()
        self.mapdata = grid
        cSpace = self.calc_cspace(grid, 1)
        self.c_space_array = cSpace
        #grid_cells = self.generateGridCells(self.mapdata, cSpace)
        #self.cspace_pub.publish(grid_cells)

    @staticmethod
    def grid_to_index(mapdata, x, y):
        """
        Returns the index corresponding to the given (x,y) coordinates in the occupancy grid.
        :param x [int] The cell X coordinate.
        :param y [int] The cell Y coordinate.
        :return  [int] The index.
        """
        i = (y * mapdata.info.width) + x
        return int (i)

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
        distance = math.sqrt(((x2 - x1) ** 2) + ((y2 - y1) ** 2))
        return distance

    @staticmethod
    def grid_to_world(mapdata, x, y):
        """
        Transforms a cell coordinate in the occupancy grid into a world coordinate.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The cell X coordinate.
        :param y       [int]           The cell Y coordinate.
        :return        [Point]         The position in the world.
        """
        initMAPX = mapdata.info.origin.position.x
        initMAPY = mapdata.info.origin.position.y
        resol = mapdata.info.resolution
        WX = ((x + 0.5) * resol + initMAPX)
        WY = ((y + 0.5) * resol + initMAPY)

        pt = Point()
        pt.x = WX
        pt.y = WY
        pt.z = 0.0
        return pt

    @staticmethod
    def world_to_grid(mapdata, wp):
        """
        Transforms a world coordinate into a cell coordinate in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param wp      [Point]         The world coordinate.
        :return        [(int,int)]     The cell position as a tuple.
        """
        WX = wp.x
        WY = wp.y
        resol = mapdata.info.resolution
        # -0.5 but coordinates to center
        gx = math.floor((WX - mapdata.info.origin.position.x) / resol - 0.5)
        gy = math.floor((WY - mapdata.info.origin.position.y) / resol - 0.5)
        return gx, gy

    @staticmethod
    def path_to_poses(mapdata, path):
        """
        Converts the given path into a list of PoseStamped.
        :param mapdata [OccupancyGrid] The map information.
        :param  path   [[(int,int)]]   The path as a list of tuples (cell coordinates).
        :return        [[PoseStamped]] The path as a list of PoseStamped (world coordinates).
        """
        p_array = []
        for pt in path:
            WP = PathPlanner.grid_to_world(mapdata, pt[1], pt[2])
            poseStamped = PoseStamped()
            poseStamped.pose.position.x = WP.x
            poseStamped.pose.position.y = WP.y
            topper = Header()
            topper.frame_id = "map"
            p_array.append(poseStamped)
            return mapdata.path_to_poses

    @staticmethod
    def is_cell_walkable(mapdata, x, y):
        """
        A cell is walkable if all of these conditions are true:
        1. It is within the boundaries of the grid;
        2. It is free (not unknown, not occupied by an obstacle)
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [boolean]       True if the cell is walkable, False otherwise
        """
        if 0 <= x < mapdata.info.width:
            if 0 <= y < mapdata.info.height:
                if 50 > mapdata.data[PathPlanner.grid_to_index(mapdata, x, y)] >= 0:
                    return True
        return False

    @staticmethod
    def neighbors_of_4(mapdata, x, y):
        """
        Returns the walkable 4-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 4-neighbors.
        """
        four_neigh = []
        if PathPlanner.is_cell_walkable(mapdata, x, y - 1):
            four_neigh.append([x, (y - 1)])
        if PathPlanner.is_cell_walkable(mapdata, x - 1, y):
            four_neigh.append([(x - 1), y])
        if PathPlanner.is_cell_walkable(mapdata, x, y + 1):
            four_neigh.append([x, (y + 1)])
        if PathPlanner.is_cell_walkable(mapdata, x + 1, y):
            four_neigh.append([(x + 1), y])

        return four_neigh

    @staticmethod
    def neighbors_of_8(mapdata, x, y):
        """
        Returns the walkable 8-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 8-neighbors.
        """
        eight_neigh = []
        if PathPlanner.is_cell_walkable(mapdata, x, y - 1):
            eight_neigh.append([x, (y - 1)])
        if PathPlanner.is_cell_walkable(mapdata, x - 1, y):
            eight_neigh.append([(x - 1), y])
        if PathPlanner.is_cell_walkable(mapdata, x, y + 1):
            eight_neigh.append([x, (y + 1)])
        if PathPlanner.is_cell_walkable(mapdata, x + 1, y):
            eight_neigh.append([(x + 1), y])
        if PathPlanner.is_cell_walkable(mapdata, x + 1, y + 1):
            eight_neigh.append([(x + 1), (y + 1)])
        if PathPlanner.is_cell_walkable(mapdata, x - 1, y - 1):
            eight_neigh.append([(x - 1), (y - 1)])
        if PathPlanner.is_cell_walkable(mapdata, x + 1, y - 1):
            eight_neigh.append([(x + 1), (y - 1)])
        if PathPlanner.is_cell_walkable(mapdata, x - 1, y + 1):
            eight_neigh.append([(x - 1), (y + 1)])

        return eight_neigh

    @staticmethod
    def request_map():
        """
        Requests the map from the map server.
        :return [OccupancyGrid] The grid if the service call was successful,
                                None in case of error.
        """

        rospy.loginfo("Requesting the map")
        rospy.wait_for_service('static_map')
        getMap = rospy.ServiceProxy('static_map', GetMap)
        g = getMap().map
        return g

    def calc_cspace(self, mapdata, padding):
        """
        Calculates the C-Space, i.e., makes the obstacles in the map thicker.
        Publishes the list of cells that were added to the original map.
        :param mapdata [OccupancyGrid] The map data.
        :param padding [int]           The number of cells around the obstacles.
        :return        [OccupancyGrid] The C-Space.
        """
        rospy.loginfo("Calculating C-Space")
        #cSpace = list(mapdata.data)
        c_Space = OccupancyGrid()
        c_Space.header = mapdata.header
        c_Space.info = mapdata.info
        c_Space.data = list(mapdata.data)
        px = 0
        py = 0
        gridCells = GridCells()
        gridCells.header.frame_id = 'map'
        gridCells.cell_width = mapdata.info.resolution
        gridCells.cell_height = mapdata.info.resolution
        gridCells.cells = []
        for node in mapdata.data:
            #if node == 100:           
                #if px == mapdata.info.width:
                #rospy.loginfo(str(len(neighbor)))
            if node > 75:              
                neighbor = PathPlanner.neighbors_of_8(mapdata, px, py)
                for those in neighbor:
                    gridCells.cells.append(PathPlanner.grid_to_world(mapdata, those[0], those[1]))
                    c_Space.data[PathPlanner.grid_to_index(mapdata, those[0], those[1])] = 100
                #cSpace[PathPlanner.grid_to_index(mapdata, px, py)]
        self.cspace_pub.publish(gridCells)
        
        return c_Space

    def convert_to_tree(self, mapdata, start, goal):
        #making a tree from graph in dictionary form
        tree = {}

        for i in range(0,mapdata.info.width):
            for j in range(0,mapdata.info.height):
                
                child_heur = {}
                
                index = self.grid_to_index(mapdata, j, i)
                
                neighbors = self.neighbors_of_8(mapdata, j, i)

                #Calculating the heuristic distances to each child node as straight line (optional landmark later)
                cost_dist = 1
                heuristic = []
                cost = []
                fn_data = []
                for k in range(0,len(neighbors)):
                    heur_dist = self.euclidean_distance(goal[0],goal[1],neighbors[k][0], neighbors[k][1])
                    heuristic.append(heur_dist)
                    if neighbors[k][0] != j and neighbors[k][1] != i:
                        cost_dist = 2
                    cost.append(cost_dist)
                    fn_data = [heuristic[k], cost[k], False]
                    child_heur[tuple(neighbors[k])] = fn_data
                    
                tree[index] = child_heur
                # print (j,i), index, tree[index]
                
        return tree

    def handle_a_star(self, req):

        'inside a star handle'

        start = req.start
        goal = req.goal

        start_x = start.pose.position.x
        start_y = start.pose.position.y

        goal_x = goal.pose.position.x
        goal_y = goal.pose.position.y

        start_point = Point()

        start_point.x=start_x
        start_point.y = start_y

        mapdata = PathPlanner.request_map()

        start = self.world_to_grid(mapdata, start_point)

        p = []

        end_point = Point()
        end_point.x = goal_x
        end_point.y = goal_y

        goal = self.world_to_grid(mapdata, end_point)


        a_path = self.a_star(mapdata, start, goal)

        world_path = []
        world_point = Point()

        print self.path

        for i in range(len(self.path)):
            world_point = self.grid_to_world(mapdata, self.path[i][0], self.path[i][1])
            world_path.append((world_point.x, world_point.y))

        print world_path
        

        for i in range(len(world_path)):
            wp = PoseStamped()
            wp.header.frame_id="odom"
            wp.pose.position.x = world_path[i][0]
            wp.pose.position.y = world_path[i][1]
            if i > 0:
                ort = math.atan2(world_path[i][1] - world_path[i-1][1], world_path[i][0] - world_path[i-1][0])
                wp.pose.orientation.w = math.cos(ort/2)
                wp.pose.orientation.z = math.sin(ort/2)
            p.append(wp)

        path = Path()
        path.header.frame_id="map"
        path.poses = p
        # print path
        return path 

    def a_star(self, mapdata, start, goal):
        ### REQUIRED CREDIT
  
        start_index_tree = self.grid_to_index(mapdata, start[0], start[1]) 

        tree = self.convert_to_tree(mapdata, start, goal)                  
        

        temp_index_tree = start_index_tree
        
        next_node = start

        path = []

        track_node = {}

        visited = [start]   

        # if (mapdata.data[self.grid_to_index(mapdata, goal[0], goal[1])]) == 100:
        #     print 'invalid goal position'


        for i in range(len(tree)):
            # print i
            if i == temp_index_tree:
                child_nodes = tree[i]
                count = 0
                
                for node in child_nodes:
                    # for visited_node in visited:
                    #     if node == visited_node:
                    #         child_nodes[node][2] = True
                    for path_node in self.path:
                        if node == path_node:
                            child_nodes[node][2] = True
                
                track_node = {}
                # print next_node, temp_index_tree
                for node in child_nodes:
                    
                    if  not child_nodes[node][2]:
                        fn = child_nodes[node][0] + child_nodes[node][1]
                        track_node[node] = fn
                        
                        # print 'child nodes', child_nodes
                        # print 'track nodes', track_node

                        next_node = min(track_node.keys(), key=(lambda k: track_node[k]))
                        visited.append(next_node)
                        

                self.path.append(next_node)
                # print self.path
                # print '---------'

                if next_node[0] == goal[0] and next_node[1] == goal[1]:
                    print 'in break'
                    break

                temp_index_tree = self.grid_to_index(mapdata, next_node[0], next_node[1])
                
                if temp_index_tree < start_index_tree:
                    # print 'in recursion'
                    self.a_star(mapdata, self.path[-1], goal)

                if next_node[0] == goal[0] and next_node[1] == goal[1]:
                    path = self.path
                    self.path = 0
                    return path
                   

                

    @staticmethod
    def optimize_path(path):
        """
        Optimizes the path, removing unnecessary intermediate nodes.
        :param path [[(x,y)]] The path as a list of tuples (grid coordinates)
        :return     [[(x,y)]] The optimized path as a list of tuples (grid coordinates)
        """
        # EXTRA CREDIT
        rospy.loginfo("Optimizing path")
        return path

    def path_to_message(self, mapdata, path):
        """
        Takes a path on the grid and returns a Path message.
        :param path [[(int,int)]] The path on the grid (a list of tuples)
        :return     [Path]        A Path message (the coordinates are expressed in the world)
        """
        p_array = []
        for pt in path:
            WP = PathPlanner.grid_to_world(mapdata, pt[1], pt[2])
            poseStamped = PoseStamped()
            poseStamped.pose.position.x = WP.x
            poseStamped.pose.position.y = WP.y
            topper = Header()
            topper.frame_id = "map"
            p_array.append(poseStamped)
        pathTopper = Header()
        pathTopper.frame_id = "map"
        pathObj = Path()
        pathObj.topper = pathTopper
        pathObj.poses = p_array
        rospy.loginfo("Returning a Path message")
        return pathObj

    def plan_path(self, msg):
        """
        Plans a path between the start and goal locations in the requested.
        Internally uses A* to plan the optimal path.
        :param req 
        """
        # Request the map
        # In case of error, return an empty path
        mapdata = PathPlanner.request_map()
        print mapdata
        if mapdata is None:
            return Path()
        # Calculate the C-space and publish it
        cspacedata = self.calc_cspace(mapdata, 1)
        # Execute A*
        start = PathPlanner.world_to_grid(mapdata, msg.start.pose.position)
        goal = PathPlanner.world_to_grid(mapdata, msg.goal.pose.position)
        print(goal)
        initPt = Point()
        initPt.x = start[0]
        initPt.y = start[1]
        goalPt = Point()
        goalPt.x = goal[0]
        goalPt.y = goal[1]
        print 'pose',   (start, goal)
        path = self.a_star(cspacedata, start, goal) #, self.c_space_array, self.frontier, self.expanded)
        print path

        # Optimize waypoints
        waypoints = PathPlanner.optimize_path(path)
        # Return a Path message
        return self.path_to_message(mapdata, waypoints)

    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        
        rospy.spin()

if __name__ == '__main__':
    PathPlanner().run()
