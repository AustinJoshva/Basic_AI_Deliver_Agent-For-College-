Project 1: Autonomous Delivery Agent

Overview:

This project implements an autonomous delivery agent that navigates a 2D grid-based city. The agent uses a combination of pathfinding algorithms to find optimal or near-optimal routes, handle various terrains, and adapt to dynamic, unexpected obstacles. The performance of each algorithm is compared experimentally across different map sizes.

Project Components:

environment.py: Defines the 2D grid environment, including static obstacles, varying terrain costs, and the logic for adding dynamic obstacles.

planner.py: Contains the core logic for the search algorithms: Breadth-First Search (BFS), A* with the Manhattan distance heuristic, and a Simulated Annealing-based replanning strategy.

experiment_runner.py: The main script used to run the experiments, generate the test maps, and output the performance metrics for each algorithm.

Getting Started:

Prerequisites:

This project requires Python 3. You do not need to install any external libraries, as all the code is built using standard Python modules.

How to Run the Experiments:
To run the experiments and generate the performance data for the project report, simply execute the experiment_runner.py script from your command-line interface (CLI).

python experiment_runner.py

The script will automatically run tests on four different map instances (small, medium, large, and dynamic) and print a comprehensive table of results to the console.

Proof-of-Concept: Dynamic Replanning
As required by the project specifications, the experiment_runner.py script includes a proof-of-concept for dynamic replanning. The script will first find a path using A*, then an unexpected obstacle will appear on the planned route, forcing the agent to detect the new obstacle and initiate a replanning strategy to find a new, valid path to its destination. This event is logged directly to the console.

Tests and Reproducibility:
The project's experiments are reproducible by running the experiment_runner.py script. The maps are generated randomly, but the seed can be set for consistent results across multiple runs.
