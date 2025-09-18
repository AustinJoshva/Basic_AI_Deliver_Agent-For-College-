import collections
import heapq
import math
import random
import time
from environment import GridEnvironment, Agent # Assuming environment.py is in the same directory

# Helper function to reconstruct the path from a dictionary of predecessors
def reconstruct_path(came_from, current):
    """
    Reconstructs the path from a dictionary of predecessors.
    
    Args:
        came_from (dict): A dictionary mapping a node to the node that preceded it on the path.
        current (tuple): The goal node.
        
    Returns:
        list: A list of (x, y) tuples representing the path from start to goal.
    """
    path = []
    # Start from the goal and trace back to the start
    while current is not None:
        path.append(current)
        current = came_from.get(current)
    path.reverse()
    return path

class Planner:
    """
    A class containing various search algorithms for pathfinding.
    """
    def __init__(self, environment):
        """
        Initializes the planner with a grid environment.
        
        Args:
            environment (GridEnvironment): The grid environment to plan on.
        """
        self.environment = environment
    
    def get_neighbors(self, x, y):
        """
        Finds all valid 4-connected neighbors of a cell.
        
        Args:
            x (int): The x-coordinate of the cell.
            y (int): The y-coordinate of the cell.
        
        Returns:
            list: A list of valid neighbor (x, y) tuples.
        """
        neighbors = []
        possible_moves = [(0, 1), (0, -1), (1, 0), (-1, 0)]  # Up, Down, Right, Left
        
        for dx, dy in possible_moves:
            nx, ny = x + dx, y + dy
            # Check if the new position is within bounds and not an obstacle
            if self.environment.is_valid_position(nx, ny) and \
               self.environment.get_cell_cost(nx, ny) != float('inf'):
                neighbors.append((nx, ny))
        return neighbors

    def bfs(self, start, goal):
        """
        Performs Breadth-First Search (BFS) to find the shortest path.
        
        Args:
            start (tuple): The starting (x, y) coordinates.
            goal (tuple): The goal (x, y) coordinates.
            
        Returns:
            tuple: A tuple containing the path (list), path cost (float), and
                   nodes expanded (int). Returns None if no path is found.
        """
        queue = collections.deque([start])
        visited = {start}
        came_from = {start: None}
        nodes_expanded = 0

        while queue:
            nodes_expanded += 1
            current = queue.popleft()

            if current == goal:
                path = reconstruct_path(came_from, current)
                path_cost = sum(self.environment.get_cell_cost(x, y) for x, y in path)
                return path, path_cost, nodes_expanded

            for neighbor in self.get_neighbors(*current):
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append(neighbor)
                    came_from[neighbor] = current
        
        return None, float('inf'), nodes_expanded # No path found

    def a_star(self, start, goal):
        """
        Performs A* search to find the optimal path.
        
        Args:
            start (tuple): The starting (x, y) coordinates.
            goal (tuple): The goal (x, y) coordinates.
            
        Returns:
            tuple: A tuple containing the path (list), path cost (float), and
                   nodes expanded (int). Returns None if no path is found.
        """
        # A* uses a priority queue for nodes to visit
        # The priority is determined by f_score = g_score + h_score
        
        open_set = [(self.heuristic(start, goal), start)] # (f_score, node)
        came_from = {start: None}
        
        g_score = {start: 0} # Cost from start to the current node
        f_score = {start: self.heuristic(start, goal)} # Estimated cost from start to goal through current node
        
        nodes_expanded = 0
        
        while open_set:
            nodes_expanded += 1
            current_f_score, current = heapq.heappop(open_set)

            if current == goal:
                path = reconstruct_path(came_from, current)
                path_cost = g_score[current]
                return path, path_cost, nodes_expanded

            for neighbor in self.get_neighbors(*current):
                tentative_g_score = g_score[current] + self.environment.get_cell_cost(*current)
                
                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    # This path to neighbor is better than any previous one. Record it!
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    
                    # Ensure neighbor is not already in the open set
                    if (f_score[neighbor], neighbor) not in open_set:
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))
                        
        return None, float('inf'), nodes_expanded # No path found

    def heuristic(self, a, b):
        """
        Calculates the Manhattan distance heuristic.
        
        Args:
            a (tuple): The starting (x, y) coordinates.
            b (tuple): The goal (x, y) coordinates.
            
        Returns:
            float: The Manhattan distance.
        """
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def simulated_annealing(self, start, goal, initial_temp=1000, cooling_rate=0.99):
        """
        Performs Simulated Annealing to find a path.
        
        This is a local search method, not a global pathfinder like A*. It's
        best for situations where a global path is not required, or for quick
        local replanning. The 'path' it returns is a single step towards the goal.
        
        Args:
            start (tuple): The starting (x, y) coordinates.
            goal (tuple): The goal (x, y) coordinates.
            initial_temp (int): The starting temperature for annealing.
            cooling_rate (float): The rate at which the temperature decreases.
            
        Returns:
            tuple: A tuple containing the next position to move to, the cost,
                   and the number of iterations.
        """
        current_pos = start
        current_cost = self.heuristic(current_pos, goal)
        temp = initial_temp
        nodes_expanded = 0
        path = [start]

        while temp > 1 and current_pos != goal:
            nodes_expanded += 1
            
            # Get a random neighbor
            neighbors = self.get_neighbors(*current_pos)
            if not neighbors:
                return None, float('inf'), nodes_expanded
            
            next_pos = random.choice(neighbors)
            next_cost = self.heuristic(next_pos, goal)

            # Decide whether to move to the new position
            delta_cost = next_cost - current_cost
            if delta_cost < 0:
                # If the new position is better, accept it
                current_pos = next_pos
                current_cost = next_cost
                path.append(current_pos)
            else:
                # If the new position is worse, accept it with a certain probability
                # This helps to escape local minima
                probability = math.exp(-delta_cost / temp)
                if random.random() < probability:
                    current_pos = next_pos
                    current_cost = next_cost
                    path.append(current_pos)
            
            # Cool down the temperature
            temp *= cooling_rate
            
        final_cost = sum(self.environment.get_cell_cost(x, y) for x, y in path)
        return path, final_cost, nodes_expanded

# Example usage:
if __name__ == "__main__":
    # Create a simple 10x10 environment
    env = GridEnvironment(width=10, height=10)
    
    # Add some obstacles
    env.add_static_obstacle(5, 5)
    env.add_static_obstacle(6, 5)
    env.add_static_obstacle(7, 5)
    
    # Add a different terrain
    env.grid[8][8] = 5
    
    start_pos = (0, 0)
    goal_pos = (9, 9)
    
    planner = Planner(env)
    
    print("--- Testing Uninformed Search (BFS) ---")
    start_time = time.time()
    bfs_path, bfs_cost, bfs_nodes = planner.bfs(start_pos, goal_pos)
    end_time = time.time()
    if bfs_path:
        print("BFS Path found!")
        print(f"Path length: {len(bfs_path)}")
        print(f"Path cost: {bfs_cost}")
        print(f"Nodes expanded: {bfs_nodes}")
        print(f"Time taken: {end_time - start_time:.4f}s")
    else:
        print("No path found by BFS.")
        
    print("-" * 20)
        
    print("--- Testing Informed Search (A*) ---")
    start_time = time.time()
    a_star_path, a_star_cost, a_star_nodes = planner.a_star(start_pos, goal_pos)
    end_time = time.time()
    if a_star_path:
        print("A* Path found!")
        print(f"Path length: {len(a_star_path)}")
        print(f"Path cost: {a_star_cost}")
        print(f"Nodes expanded: {a_star_nodes}")
        print(f"Time taken: {end_time - start_time:.4f}s")
    else:
        print("No path found by A*.")
        
    print("-" * 20)
    
    print("--- Testing Local Search (Simulated Annealing) ---")
    sa_path, sa_cost, sa_nodes = planner.simulated_annealing(start_pos, goal_pos)
    if sa_path:
        print("Simulated Annealing path found!")
        print(f"Path length: {len(sa_path)}")
        print(f"Path cost: {sa_cost}")
        print(f"Iterations: {sa_nodes}")
    else:
        print("Simulated Annealing failed to find a path.")
        
    print("-" * 20)
        
    print("--- Dynamic Replanning Demonstration ---")
    # Initial plan with A*
    agent_pos = start_pos
    current_path, _, _ = planner.a_star(agent_pos, goal_pos)
    if not current_path:
        print("Initial path could not be found.")
    else:
        print("Initial path found using A*.")
        
        # Simulate moving a few steps along the path
        next_step = current_path[2]
        print(f"Agent moves from {agent_pos} to {next_step}.")
        agent_pos = next_step
        
        # Dynamically add a new obstacle on the path
        new_obstacle_pos = current_path[3]
        print(f"Dynamically adding a new obstacle at {new_obstacle_pos}.")
        env.add_static_obstacle(new_obstacle_pos[0], new_obstacle_pos[1])
        
        # Agent detects the obstacle and replans from its current position
        print(f"Agent detects an obstacle and must replan from {agent_pos}.")
        replanned_path, replanned_cost, _ = planner.a_star(agent_pos, goal_pos)
        
        if replanned_path:
            print("Replanning successful!")
            print(f"New path length: {len(replanned_path)}")
            print(f"New path cost: {replanned_cost}")
        else:
            print("Replanning failed. No new path found.")
