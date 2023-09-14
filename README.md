# MDP_Algorithm
Autonomous Robot Navigation Algorithms
This repository contains the Java algorithms I developed for enabling efficient navigation of an autonomous robot through mazes during an 8-week Multidisciplinary Design Project.

# Implementation
The core path planning algorithm is implemented in Java based on A* search to find the shortest path between the start and goal positions, avoiding obstacles.

## Key Components
AStarSearch.java - Contains the main A* search logic including computing the open and closed lists, selecting the next node, calculating heuristics etc.
Node.java - Represents a node on the grid map, containing coordinates, parent node reference, cost values etc.
Map.java - Handles the grid representation and obstacle positions. Provides method to check if a node is valid.
Heuristic.java - Calculates the heuristic cost between two node positions using the Manhattan distance.
Path.java - Reconstructs the final path after reaching the goal by traversing back using parent node pointers.


# Flow
The high level flow is:

Initialize the grid Map with obstacles
Add the start node to the open list
Loop until open list is empty:
Remove the node with lowest fCost from open list
Check if it is the goal node, if yes reconstruct path
Else expand its neighbors, calculate costs and add valid ones to open list
Reconstruct optimal path by traversing parent nodes


# Usage
The main method in AStarSearch shows sample usage:

Create a Map instance and add obstacles
Create a AStarSearch instance with the Map
Call findPath() by passing start and end positions
Get the path list containing nodes from start to goal

# Robot Integration
The algorithm code was integrated with the robot's hardware and software systems:

Raspberry Pi server running the motor control program
Motor control circuits to enable directional driving of the wheels
Ultrasonic and infrared sensors to detect obstacles
Camera for vision and image processing tasks
Extensive testing was done to refine the algorithms before the final maze navigation challenge. The optimized navigation logic allowed the robot to quickly traverse unknown mazes with 100% accuracy, leading to a 2nd place among over 270 teams.

# Usage
The Java files are well documented for understanding the core logic and integration. Key areas to review:

Astar folder - Implements optimized depth-first search and can be modified to account for thr various turns thre robot makes
Server folder - Communicates with the Raspberry Pi server and initialises the server
