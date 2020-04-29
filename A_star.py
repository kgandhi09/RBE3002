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
                    fn_data = [heuristic[k], cost[k]]
                    child_heur[tuple(neighbors[k])] = fn_data
                    
                tree[index] = child_heur
        return tree


def a_star(self, mapdata, start, goal):
    ### REQUIRED CREDIT

    start_index_tree = self.grid_to_index(mapdata, start[0], start[1]) 

    tree = self.convert_to_tree(mapdata, start, goal)                  

    temp_index_tree = start_index_tree
    
    fn_child_node = []

    track_node = []

    # if (mapdata.data[self.grid_to_index(mapdata, goal[0], goal[1])]) == 100:
    #     print 'invalid goal position'

    for i in range(len(tree)):
        if i == temp_index_tree:
            child_nodes = tree[i]
            count = 0
            # print child_nodes
            for node in child_nodes:
                fn = child_nodes[node][0] + child_nodes[node][1]        #f(n) = g(n) + h(n)
                track_node.append(child_nodes[node][0])
                fn_child_node.append(fn)
                
            min_index = fn_child_node.index(min(fn_child_node))
            min_child_index = track_node[min_index]
        
            for node in child_nodes:
                if min_child_index == child_nodes[node][0] and count == 0:
                    
                    next_node = node
                    self.path.append(next_node)
                    temp_index_tree = self.grid_to_index(mapdata, next_node[0], next_node[1])
                    
                    if self.path != [] and temp_index_tree < start_index_tree:
                        print next_node[0], next_node[1], goal[0], goal[1]
                        if next_node[0] != goal[0] and next_node[1] != goal[1]:
                            self.a_star(mapdata, next_node, goal)
                    count = 1
                    
                

    return self.path