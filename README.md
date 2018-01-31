# A-algorithm-Python
Implementation of the A* ssearch algorithm in Python

This is an implementation of the A* search algorithm. It uses the Euclidean distance as the heuristic.
The graph information is read from a .lp file (there are some samples to guide users about the structure of the file).

The actual algorithm is present in the 'solution.py' file, whereas 'parsing.py' processes the .lp file and acquires the necessary information from the graph (start, end, coordinates of each node, adjacent nodes for each node). 'run_astar.py' is a wrapper for the algorithm function present in tthe 'solution.py' file, and can be executed from the python shell to try the algorithm out.
