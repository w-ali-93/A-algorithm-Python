from queue import PriorityQueue
import math
from parsing import read_instance


def astar(coords, adj, start, goal):
    # Nodes are numbered from 1 to n; 
    # 0 is a dummy node with no edges
    n = len(coords) - 1
    
    # Initialize tables
    
    # prev[x]: the previous node on the
    # shortest path found so far from start to x
    prev = [i for i in range(n+1)]
    mindist = [math.inf for i in range(n+1)]

    # Heuristic function
    def heur(pos1, pos2):
        x1,y1 = pos1
        x2,y2 = pos2
        #return min(abs(x1 - x2), abs(y1 - y2)) # works but higher computation time as compared to Euclidean distance
        #return abs(x1-x2) + abs(y1-y2) # Manhattan distance - does not guarantee admissibility
        return math.sqrt(abs(x1-x2)*abs(x1-x2) + abs(y1-y2)*abs(y1-y2))  # Euclidean distance - works and fastest


    # Memoization: compute heuristics on demand and only once
    _heur = [-1] * (n+1)
    def get_heur(i):
        if _heur[i] == -1:
            _heur[i] = heur(coords[i],coords[goal])
        return _heur[i]

    # Unwind path from prev[x] table
    def reconstruct_path(came_from, start, goal):
        current = goal
        path = []
        while current != start:
            path.append(current)
            current = came_from[current]
        path.append(start)
        path.reverse()
        return path

    # Initialize A* algorithm
    queue=PriorityQueue()
    found = False
    mindist[start]=0
    queue.put((0, start))

    # Main loop for processing the queue
    while not queue.empty():
        (_, node) = queue.get() # get the next node in the priority queue
        if node == goal: # if this is the goal node
            found = True # set found flag as true
            break # exit loop execution
        # found the optimal path from start to goal
        for (next,l) in adj[node]:
            # next: a neighbor of 'node'
            # l: the distance from 'node' to 'next'
            new_cost = mindist[node] + l; # add distance from current node to neighbor into the travel cost
            if new_cost < mindist[next]: # if the newly computed cost is less than minimum distance for travelling to the neighbor node
                mindist[next] = new_cost # set newly computed cost as the minimum distance
                prev[next] = node # set the current node as the previous node for the neighbor
                queue.put((mindist[next] + get_heur(next), next)) # put the neighbor into the priority queue alongside the sum of its travel cost and heuristic cost as its priority
    # If the goal was found, unwind the path
    path = reconstruct_path(prev, start, goal)

    if found:
        return path
    else:
        return None
