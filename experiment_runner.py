import time
import random
from environment import GridEnvironment
from planner import Planner

def generate_map(width, height, obstacle_density=0.2):

    env = GridEnvironment(width, height)
    num_obstacles = int(width * height * obstacle_density)

    for _ in range(num_obstacles):
        x = random.randint(0, width - 1)
        y = random.randint(0, height - 1)
        env.add_static_obstacle(x, y)

    for _ in range(int(width * height * 0.1)):
        x = random.randint(0, width - 1)
        y = random.randint(0, height - 1)
        env.grid[y][x] = random.randint(2, 5)

    return env

def generate_dynamic_map(width, height):

    env = generate_map(width, height, obstacle_density=0.1)
    
    dynamic_schedule = []

    for i in range(5): dynamic_schedule.append((i, 0))
    for i in range(5): dynamic_schedule.append((4, i))
    for i in range(5, 0, -1): dynamic_schedule.append((i, 4))
    for i in range(5, 0, -1): dynamic_schedule.append((0, i))
    env.add_dynamic_obstacle(0, 0, schedule=dynamic_schedule)

    return env

def run_experiment(planner, env, start, goal):

    results = {}
    start_time = time.time()
    path, cost, nodes = planner.bfs(start, goal)
    end_time = time.time()
    results['bfs'] = {
        'path_cost': cost,
        'nodes_expanded': nodes,
        'time': end_time - start_time,
        'path_found': path is not None
    }

    
    start_time = time.time()
    path, cost, nodes = planner.a_star(start, goal)
    end_time = time.time()
    results['a_star'] = {
        'path_cost': cost,
        'nodes_expanded': nodes,
        'time': end_time - start_time,
        'path_found': path is not None
    }

    start_time = time.time()
    path, cost, nodes = planner.simulated_annealing(start, goal)
    end_time = time.time()
    results['sa'] = {
        'path_cost': cost,
        'nodes_expanded': nodes,
        'time': end_time - start_time,
        'path_found': path is not None
    }
    
    return results

def print_results(results, map_name):
    print(f"\n--- Results for {map_name} ---")
    print("{:<20} {:<15} {:<15} {:<15}".format("Algorithm", "Path Cost", "Nodes Exp.", "Time (s)"))
    print("-" * 65)
    for algo, data in results.items():
        cost = f"{data['path_cost']:.2f}" if data['path_found'] else "N/A"
        time_taken = f"{data['time']:.4f}"
        print("{:<20} {:<15} {:<15} {:<15}".format(
            algo,
            cost,
            data['nodes_expanded'],
            time_taken
        ))

if __name__ == "__main__":
    small_size = (10, 10)
    medium_size = (20, 20)
    large_size = (50, 50)
    
    print("Starting Experiments...")

    small_map = generate_map(*small_size)
    planner_small = Planner(small_map)
    small_results = run_experiment(planner_small, small_map, (0, 0), (9, 9))
    print_results(small_results, "Small Map")

    medium_map = generate_map(*medium_size)
    planner_medium = Planner(medium_map)
    medium_results = run_experiment(planner_medium, medium_map, (0, 0), (19, 19))
    print_results(medium_results, "Medium Map")


    large_map = generate_map(*large_size)
    planner_large = Planner(large_map)
    large_results = run_experiment(planner_large, large_map, (0, 0), (49, 49))
    print_results(large_results, "Large Map")

    dynamic_map = generate_dynamic_map(20, 20)
    planner_dynamic = Planner(dynamic_map)
    dynamic_results = run_experiment(planner_dynamic, dynamic_map, (0, 0), (19, 19))
    print_results(dynamic_results, "Dynamic Map")
    
    print("\nExperiments complete. You can use these results for your report.")
    
