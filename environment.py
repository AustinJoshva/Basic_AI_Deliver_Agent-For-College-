import random

class GridEnvironment:
    def __init__(self, width, height, default_cost=1):
        self.width = width
        self.height = height
        self.grid = [[default_cost for _ in range(width)] for _ in range(height)]
        self.static_obstacles = []
        self.dynamic_obstacles = []

    def add_static_obstacle(self, x, y, cost=float('inf')):
        if 0 <= y < self.height and 0 <= x < self.width:
            self.grid[y][x] = cost
            self.static_obstacles.append((x, y))

    def add_dynamic_obstacle(self, x, y, schedule=None):
        obstacle = {'position': (x, y), 'schedule': schedule if schedule else []}
        self.dynamic_obstacles.append(obstacle)

    def is_valid_position(self, x, y):
        return 0 <= x < self.width and 0 <= y < self.height

    def get_cell_cost(self, x, y):
    
        if self.is_valid_position(x, y):
            return self.grid[y][x]
        return float('inf')

    def get_dynamic_obstacle_positions(self, time_step=0):
        positions = []
        for obstacle in self.dynamic_obstacles:
            schedule = obstacle['schedule']
            if schedule and time_step < len(schedule):
                positions.append(schedule[time_step])
            else:
                positions.append(obstacle['position'])
        return positions

class Agent:

    def __init__(self, start_position, environment):
 
        self.position = start_position
        self.environment = environment
        self.path = []
        self.cost = 0

    def move(self, new_position):

        x, y = new_position
        if self.environment.is_valid_position(x, y) and \
           self.environment.get_cell_cost(x, y) != float('inf'):
            self.path.append(new_position)
            self.cost += self.environment.get_cell_cost(x, y)
            self.position = new_position
            return True
        return False


if __name__ == "__main__":
    
    env = GridEnvironment(width=10, height=10)

    env.add_static_obstacle(2, 2)
    env.add_static_obstacle(3, 4)

    env.grid[5][5] = 3 
    env.grid[5][6] = 3

    dynamic_schedule = [(1, 1), (1, 2), (1, 3)]
    env.add_dynamic_obstacle(1, 1, schedule=dynamic_schedule)

    agent = Agent(start_position=(0, 0), environment=env)

    print(f"Agent starting at: {agent.position}")
    print(f"Initial cost: {agent.cost}")

    if agent.move((1, 0)):
        print(f"Agent moved to: {agent.position}, current cost: {agent.cost}")

    if not agent.move((2, 2)):
        print("Agent could not move into an obstacle.")

    print(f"Dynamic obstacle at time 0: {env.get_dynamic_obstacle_positions(time_step=0)}")
    print(f"Dynamic obstacle at time 1: {env.get_dynamic_obstacle_positions(time_step=1)}")
