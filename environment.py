import random

class GridEnvironment:
    """
    Represents the 2D grid city environment for the delivery agent.

    The grid is a list of lists, where each cell contains a movement cost.
    Static obstacles are represented by a high movement cost.
    Dynamic obstacles are stored in a separate list with their positions.
    """
    def __init__(self, width, height, default_cost=1):
        """
        Initializes the grid environment.

        Args:
            width (int): The width of the grid.
            height (int): The height of the grid.
            default_cost (int): The default movement cost for a cell.
        """
        self.width = width
        self.height = height
        self.grid = [[default_cost for _ in range(width)] for _ in range(height)]
        self.static_obstacles = []
        self.dynamic_obstacles = []

    def add_static_obstacle(self, x, y, cost=float('inf')):
        """
        Adds a static obstacle to the grid.

        Args:
            x (int): The x-coordinate of the obstacle.
            y (int): The y-coordinate of the obstacle.
            cost (float): The cost to move through the obstacle (default is infinity).
        """
        if 0 <= y < self.height and 0 <= x < self.width:
            self.grid[y][x] = cost
            self.static_obstacles.append((x, y))

    def add_dynamic_obstacle(self, x, y, schedule=None):
        """
        Adds a dynamic obstacle with a potential schedule.

        Args:
            x (int): The initial x-coordinate.
            y (int): The initial y-coordinate.
            schedule (list, optional): A list of future positions for a deterministic obstacle.
        """
        obstacle = {'position': (x, y), 'schedule': schedule if schedule else []}
        self.dynamic_obstacles.append(obstacle)

    def is_valid_position(self, x, y):
        """
        Checks if a position is within the grid boundaries.

        Args:
            x (int): The x-coordinate.
            y (int): The y-coordinate.

        Returns:
            bool: True if the position is valid, False otherwise.
        """
        return 0 <= x < self.width and 0 <= y < self.height

    def get_cell_cost(self, x, y):
        """
        Returns the movement cost of a specific cell.

        Args:
            x (int): The x-coordinate.
            y (int): The y-coordinate.

        Returns:
            int or float: The movement cost.
        """
        if self.is_valid_position(x, y):
            return self.grid[y][x]
        return float('inf')

    def get_dynamic_obstacle_positions(self, time_step=0):
        """
        Returns the positions of all dynamic obstacles at a given time step.

        For deterministic obstacles, this uses the schedule. For unpredictable
        obstacles, it returns the current position.

        Args:
            time_step (int): The current time step.

        Returns:
            list: A list of (x, y) tuples representing obstacle positions.
        """
        positions = []
        for obstacle in self.dynamic_obstacles:
            schedule = obstacle['schedule']
            if schedule and time_step < len(schedule):
                positions.append(schedule[time_step])
            else:
                positions.append(obstacle['position'])
        return positions

class Agent:
    """
    Represents the autonomous delivery agent.
    """
    def __init__(self, start_position, environment):
        """
        Initializes the agent.

        Args:
            start_position (tuple): The starting (x, y) coordinates.
            environment (GridEnvironment): The environment the agent is in.
        """
        self.position = start_position
        self.environment = environment
        self.path = []
        self.cost = 0

    def move(self, new_position):
        """
        Moves the agent to a new position if it's a valid move.

        Args:
            new_position (tuple): The new (x, y) coordinates.
        """
        x, y = new_position
        if self.environment.is_valid_position(x, y) and \
           self.environment.get_cell_cost(x, y) != float('inf'):
            self.path.append(new_position)
            self.cost += self.environment.get_cell_cost(x, y)
            self.position = new_position
            return True
        return False

# Example usage:
if __name__ == "__main__":
    # Create a 10x10 environment
    env = GridEnvironment(width=10, height=10)

    # Add static obstacles (e.g., buildings)
    env.add_static_obstacle(2, 2)
    env.add_static_obstacle(3, 4)

    # Add varying terrain costs
    env.grid[5][5] = 3  # A dirt path
    env.grid[5][6] = 3

    # Add a dynamic obstacle with a known schedule
    # It moves from (1, 1) -> (1, 2) -> (1, 3)
    dynamic_schedule = [(1, 1), (1, 2), (1, 3)]
    env.add_dynamic_obstacle(1, 1, schedule=dynamic_schedule)

    # Initialize the agent at a starting position
    agent = Agent(start_position=(0, 0), environment=env)

    # The agent can "plan" its path here, but for now, we'll just demonstrate the movement.
    print(f"Agent starting at: {agent.position}")
    print(f"Initial cost: {agent.cost}")

    # Move the agent and observe the change in position and cost
    if agent.move((1, 0)):
        print(f"Agent moved to: {agent.position}, current cost: {agent.cost}")

    # Trying to move into a static obstacle
    if not agent.move((2, 2)):
        print("Agent could not move into an obstacle.")

    # Get dynamic obstacle positions at different time steps
    print(f"Dynamic obstacle at time 0: {env.get_dynamic_obstacle_positions(time_step=0)}")
    print(f"Dynamic obstacle at time 1: {env.get_dynamic_obstacle_positions(time_step=1)}")