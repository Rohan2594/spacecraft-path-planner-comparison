import numpy as np
import matplotlib.pyplot as plt
import math
import heapq
import time
import argparse
import os

# Redundant import as a failsafe
import numpy as np

class PathPlanner:
    def __init__(self, clearance=5, robot_radius=10, step_size=1, map_width=100, map_height=100):
        self.clearance = clearance
        self.robot_radius = robot_radius
        self.step_size = step_size
        self.total_clearance = clearance + robot_radius
        
        # Map dimensions (now configurable)
        self.map_width = map_width
        self.map_height = map_height
        
        # Initialize obstacle map
        self.obstacle_map = self.create_obstacle_map()
    
    def create_performance_chart(self, astar_result, dijkstra_result):
        """Create performance comparison bar chart"""
        plt.figure(figsize=(15, 10))
        
        # Create subplots for different metrics
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 10))
        
        algorithms = ['A*', 'Dijkstra']
        
        # 1. Nodes Explored
        nodes_data = [astar_result['nodes_explored'], dijkstra_result['nodes_explored']]
        bars1 = ax1.bar(algorithms, nodes_data, color=['blue', 'red'], alpha=0.7)
        ax1.set_title('Nodes Explored', fontweight='bold')
        ax1.set_ylabel('Number of Nodes')
        ax1.grid(True, alpha=0.3)
        # Add value labels on bars
        for bar, value in zip(bars1, nodes_data):
            ax1.text(bar.get_x() + bar.get_width()/2, bar.get_height() + max(nodes_data)*0.01,
                    str(value), ha='center', va='bottom', fontweight='bold')
        
        # 2. Time Taken
        time_data = [astar_result['time_taken'], dijkstra_result['time_taken']]
        bars2 = ax2.bar(algorithms, time_data, color=['blue', 'red'], alpha=0.7)
        ax2.set_title('Execution Time', fontweight='bold')
        ax2.set_ylabel('Time (seconds)')
        ax2.grid(True, alpha=0.3)
        for bar, value in zip(bars2, time_data):
            ax2.text(bar.get_x() + bar.get_width()/2, bar.get_height() + max(time_data)*0.01,
                    f'{value:.4f}s', ha='center', va='bottom', fontweight='bold')
        
        # 3. Path Cost
        cost_data = [astar_result['total_cost'], dijkstra_result['total_cost']]
        bars3 = ax3.bar(algorithms, cost_data, color=['blue', 'red'], alpha=0.7)
        ax3.set_title('Total Path Cost', fontweight='bold')
        ax3.set_ylabel('Cost')
        ax3.grid(True, alpha=0.3)
        for bar, value in zip(bars3, cost_data):
            ax3.text(bar.get_x() + bar.get_width()/2, bar.get_height() + max(cost_data)*0.01,
                    f'{value:.2f}', ha='center', va='bottom', fontweight='bold')
        
        # 4. Path Length
        length_data = [astar_result['path_length'], dijkstra_result['path_length']]
        bars4 = ax4.bar(algorithms, length_data, color=['blue', 'red'], alpha=0.7)
        ax4.set_title('Path Length (nodes)', fontweight='bold')
        ax4.set_ylabel('Number of Nodes')
        ax4.grid(True, alpha=0.3)
        for bar, value in zip(bars4, length_data):
            ax4.text(bar.get_x() + bar.get_width()/2, bar.get_height() + max(length_data)*0.01,
                    str(value), ha='center', va='bottom', fontweight='bold')
        
        plt.suptitle('Performance Metrics Comparison', fontsize=16, fontweight='bold')
        plt.tight_layout()
        plt.subplots_adjust(top=0.93)
        plt.show()
        
    def create_obstacle_map(self):
        """Create obstacle map with predefined obstacles scaled to map size"""
        obstacle_map = np.zeros((self.map_height, self.map_width))
        
        # Scale obstacles based on map dimensions
        scale_x = self.map_width / 100.0
        scale_y = self.map_height / 100.0
        
        # Scaled Rectangle obstacles
        rect1_x1 = int(20 * scale_x)
        rect1_x2 = int(40 * scale_x)
        rect1_y1 = int(20 * scale_y)
        rect1_y2 = int(35 * scale_y)
        obstacle_map[rect1_y1:rect1_y2, rect1_x1:rect1_x2] = 1
        
        rect2_x1 = int(60 * scale_x)
        rect2_x2 = int(85 * scale_x)
        rect2_y1 = int(60 * scale_y)
        rect2_y2 = int(80 * scale_y)
        obstacle_map[rect2_y1:rect2_y2, rect2_x1:rect2_x2] = 1
        
        rect3_x1 = int(10 * scale_x)
        rect3_x2 = int(30 * scale_x)
        rect3_y1 = int(70 * scale_y)
        rect3_y2 = int(90 * scale_y)
        obstacle_map[rect3_y1:rect3_y2, rect3_x1:rect3_x2] = 1
        
        # Scaled Circle obstacle
        center_x = int(70 * scale_x)
        center_y = int(30 * scale_y)
        radius = int(10 * min(scale_x, scale_y))
        
        for i in range(self.map_height):
            for j in range(self.map_width):
                if (i - center_y)**2 + (j - center_x)**2 <= radius**2:
                    obstacle_map[i, j] = 1
        
        # Apply clearance
        return self.apply_clearance(obstacle_map)
    
    def apply_clearance(self, obstacle_map):
        """Apply clearance around obstacles"""
        cleared_map = obstacle_map.copy()
        total_clearance_int = int(self.total_clearance)
        
        for i in range(self.map_height):
            for j in range(self.map_width):
                if obstacle_map[i, j] == 1:
                    # Apply clearance around obstacle
                    for di in range(-total_clearance_int, total_clearance_int + 1):
                        for dj in range(-total_clearance_int, total_clearance_int + 1):
                            ni, nj = i + di, j + dj
                            if (0 <= ni < self.map_height and 0 <= nj < self.map_width and
                                di**2 + dj**2 <= self.total_clearance**2):
                                if cleared_map[ni, nj] == 0:
                                    cleared_map[ni, nj] = 0.5  # Clearance space
        return cleared_map
    
    def is_valid_point(self, x, y):
        """Check if point is valid (not in obstacle or clearance)"""
        try:
            # Convert to integers for array indexing
            x_int = int(round(x))
            y_int = int(round(y))
            
            # Check bounds
            if x_int < 0 or x_int >= self.map_width or y_int < 0 or y_int >= self.map_height:
                return False
                
            # Check if point is in free space (value == 0)
            return self.obstacle_map[y_int, x_int] == 0
        except (ValueError, IndexError):
            return False
    
    def get_neighbors(self, current_state):
        """Get valid neighboring states"""
        x, y, theta = current_state
        neighbors = []
        
        # 5 action set with theta threshold of 30 degrees
        actions = [
            (0, self.step_size),      # Forward
            (-30, self.step_size),    # Forward Left
            (30, self.step_size),     # Forward Right
            (-60, self.step_size),    # Sharp Left
            (60, self.step_size)      # Sharp Right
        ]
        
        for dtheta, distance in actions:
            new_theta = (theta + dtheta) % 360
            new_x = x + distance * math.cos(math.radians(new_theta))
            new_y = y + distance * math.sin(math.radians(new_theta))
            
            # Round to avoid floating point precision issues
            new_x = round(new_x, 2)
            new_y = round(new_y, 2)
            new_theta = round(new_theta, 2)
            
            if self.is_valid_point(new_x, new_y):
                neighbors.append((new_x, new_y, new_theta, distance))
        
        return neighbors
    
    def euclidean_distance(self, state1, state2):
        """Calculate Euclidean distance between two states"""
        x1, y1 = state1[:2]
        x2, y2 = state2[:2]
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
    def is_goal_reached(self, current_state, goal_state, threshold=0.5):
        """Check if goal is reached within threshold"""
        return self.euclidean_distance(current_state, goal_state) <= threshold
    
    def reconstruct_path(self, came_from, current_state):
        """Reconstruct path from goal to start"""
        path = [current_state]
        while current_state in came_from:
            current_state = came_from[current_state]
            path.append(current_state)
        return path[::-1]
    
    def a_star_search(self, start, goal):
        """A* Algorithm Implementation"""
        print("Running A* Algorithm...")
        start_time = time.time()
        
        # Round start and goal to avoid floating point issues
        start = (round(start[0], 2), round(start[1], 2), round(start[2], 2))
        goal = (round(goal[0], 2), round(goal[1], 2), round(goal[2], 2))
        
        # Priority queue: (f_score, g_score, state)
        open_set = [(0, 0, start)]
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.euclidean_distance(start, goal)}
        
        visited = set()
        explored_nodes = []
        nodes_explored = 0
        
        while open_set:
            current_f, current_g, current_state = heapq.heappop(open_set)
            
            if current_state in visited:
                continue
                
            visited.add(current_state)
            explored_nodes.append(current_state)
            nodes_explored += 1
            
            if self.is_goal_reached(current_state, goal):
                end_time = time.time()
                path = self.reconstruct_path(came_from, current_state)
                print(f"A* - Goal reached! Path found with {len(path)} nodes")
                return {
                    'path': path,
                    'explored_nodes': explored_nodes,
                    'nodes_explored': nodes_explored,
                    'path_length': len(path),
                    'total_cost': current_g,
                    'time_taken': end_time - start_time,
                    'algorithm': 'A*'
                }
            
            for neighbor_x, neighbor_y, neighbor_theta, move_cost in self.get_neighbors(current_state):
                neighbor_state = (neighbor_x, neighbor_y, neighbor_theta)
                
                if neighbor_state in visited:
                    continue
                
                tentative_g_score = g_score[current_state] + move_cost
                
                if neighbor_state not in g_score or tentative_g_score < g_score[neighbor_state]:
                    came_from[neighbor_state] = current_state
                    g_score[neighbor_state] = tentative_g_score
                    f_score[neighbor_state] = tentative_g_score + self.euclidean_distance(neighbor_state, goal)
                    heapq.heappush(open_set, (f_score[neighbor_state], tentative_g_score, neighbor_state))
        
        end_time = time.time()
        print("A* - No path found!")
        return {
            'path': None,
            'explored_nodes': explored_nodes,
            'nodes_explored': nodes_explored,
            'path_length': 0,
            'total_cost': float('inf'),
            'time_taken': end_time - start_time,
            'algorithm': 'A*'
        }
    
    def dijkstra_search(self, start, goal):
        """Dijkstra's Algorithm Implementation"""
        print("Running Dijkstra's Algorithm...")
        start_time = time.time()
        
        # Round start and goal to avoid floating point issues
        start = (round(start[0], 2), round(start[1], 2), round(start[2], 2))
        goal = (round(goal[0], 2), round(goal[1], 2), round(goal[2], 2))
        
        # Priority queue: (distance, state)
        open_set = [(0, start)]
        came_from = {}
        distances = {start: 0}
        
        visited = set()
        explored_nodes = []
        nodes_explored = 0
        
        while open_set:
            current_distance, current_state = heapq.heappop(open_set)
            
            if current_state in visited:
                continue
                
            visited.add(current_state)
            explored_nodes.append(current_state)
            nodes_explored += 1
            
            if self.is_goal_reached(current_state, goal):
                end_time = time.time()
                path = self.reconstruct_path(came_from, current_state)
                print(f"Dijkstra - Goal reached! Path found with {len(path)} nodes")
                return {
                    'path': path,
                    'explored_nodes': explored_nodes,
                    'nodes_explored': nodes_explored,
                    'path_length': len(path),
                    'total_cost': current_distance,
                    'time_taken': end_time - start_time,
                    'algorithm': 'Dijkstra'
                }
            
            for neighbor_x, neighbor_y, neighbor_theta, move_cost in self.get_neighbors(current_state):
                neighbor_state = (neighbor_x, neighbor_y, neighbor_theta)
                
                if neighbor_state in visited:
                    continue
                
                new_distance = distances[current_state] + move_cost
                
                if neighbor_state not in distances or new_distance < distances[neighbor_state]:
                    distances[neighbor_state] = new_distance
                    came_from[neighbor_state] = current_state
                    heapq.heappush(open_set, (new_distance, neighbor_state))
        
        end_time = time.time()
        print("Dijkstra - No path found!")
        return {
            'path': None,
            'explored_nodes': explored_nodes,
            'nodes_explored': nodes_explored,
            'path_length': 0,
            'total_cost': float('inf'),
            'time_taken': end_time - start_time,
            'algorithm': 'Dijkstra'
        }
    
    def visualize_comparison(self, astar_result, dijkstra_result, start, goal):
        """Visualize both algorithms side by side"""
        plt.style.use('default')  # Ensure clean plotting style
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(20, 10))
        
        # A* Visualization
        self.plot_single_result(ax1, astar_result, start, goal, "A* Algorithm")
        
        # Dijkstra Visualization
        self.plot_single_result(ax2, dijkstra_result, start, goal, "Dijkstra's Algorithm")
        
        # Add overall comparison info
        fig.suptitle('Path Planning Algorithm Comparison: A* vs Dijkstra', fontsize=16, fontweight='bold')
        
        plt.tight_layout()
        plt.subplots_adjust(top=0.9)  # Make room for suptitle
        
        # For web app compatibility, don't show the plot here
        # plt.show() - commented out for web app usage
        
        # Also create a performance comparison bar chart if running standalone
        # self.create_performance_chart(astar_result, dijkstra_result)
    
    def plot_single_result(self, ax, result, start, goal, title):
        """Plot single algorithm result"""
        # Create color map
        obstacle_map_colored = np.zeros((self.map_height, self.map_width, 3))
        
        # White for free space
        obstacle_map_colored[self.obstacle_map == 0] = [1, 1, 1]
        # Blue for obstacles
        obstacle_map_colored[self.obstacle_map == 1] = [0, 0, 1]
        # Cyan for clearance
        obstacle_map_colored[self.obstacle_map == 0.5] = [0, 1, 1]
        
        ax.imshow(obstacle_map_colored, origin='lower', extent=[0, self.map_width, 0, self.map_height])
        
        # Plot explored nodes in green
        if result['explored_nodes']:
            explored_x = [state[0] for state in result['explored_nodes']]
            explored_y = [state[1] for state in result['explored_nodes']]
            ax.scatter(explored_x, explored_y, c='green', s=1, alpha=0.6, label='Explored Nodes')
        
        # Plot path in red (thicker and solid with markers)
        if result['path']:
            path_x = [state[0] for state in result['path']]
            path_y = [state[1] for state in result['path']]
            ax.plot(path_x, path_y, 'r-', linewidth=4, marker='o', markersize=6, label='Planned Path')
            # Annotate path length below the plot
            path_length = len(result['path'])
            ax.text(
                0.5, -0.12,
                f"Path Length: {path_length} nodes",
                transform=ax.transAxes,
                fontsize=14,
                color='red',
                ha='center',
                va='top',
                fontweight='bold'
            )
            # Add arrows to indicate direction
            if len(result['path']) > 1:
                for i in range(0, len(result['path']) - 1, max(1, len(result['path']) // 10)):
                    ax.annotate(
                        '',
                        xy=(result['path'][i+1][0], result['path'][i+1][1]),
                        xytext=(result['path'][i][0], result['path'][i][1]),
                        arrowprops=dict(arrowstyle='->', color='red', lw=2)
                    )
        
        # Plot start and goal
        ax.plot(start[0], start[1], 'go', markersize=10, label='Start')
        ax.plot(goal[0], goal[1], 'ro', markersize=10, label='Goal')
        
        ax.set_title(f'{title}\nNodes Explored: {result["nodes_explored"]}, '
                    f'Time: {result["time_taken"]:.3f}s, '
                    f'Path Cost: {result["total_cost"]:.2f}')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.legend()
        ax.grid(True, alpha=0.3)
    
    def compare_algorithms(self, start, goal):
        """Compare A* and Dijkstra algorithms"""
        print("=" * 60)
        print("ALGORITHM COMPARISON: A* vs Dijkstra")
        print("=" * 60)
        
        # Run both algorithms
        astar_result = self.a_star_search(start, goal)
        dijkstra_result = self.dijkstra_search(start, goal)
        
        # Print comparison results
        self.print_comparison_results(astar_result, dijkstra_result)
        
        # Visualize results
        self.visualize_comparison(astar_result, dijkstra_result, start, goal)
        
        return astar_result, dijkstra_result
    
    def print_comparison_results(self, astar_result, dijkstra_result):
        """Print detailed comparison results"""
        print("\nCOMPARISON RESULTS:")
        print("-" * 60)
        
        # Table header
        print(f"{'Metric':<25} {'A*':<15} {'Dijkstra':<15} {'Winner':<10}")
        print("-" * 60)
        
        # Nodes explored
        astar_nodes = astar_result['nodes_explored']
        dijkstra_nodes = dijkstra_result['nodes_explored']
        nodes_winner = "A*" if astar_nodes < dijkstra_nodes else "Dijkstra" if dijkstra_nodes < astar_nodes else "Tie"
        print(f"{'Nodes Explored':<25} {astar_nodes:<15} {dijkstra_nodes:<15} {nodes_winner:<10}")
        
        # Time taken
        astar_time = astar_result['time_taken']
        dijkstra_time = dijkstra_result['time_taken']
        time_winner = "A*" if astar_time < dijkstra_time else "Dijkstra" if dijkstra_time < astar_time else "Tie"
        print(f"{'Time Taken (s)':<25} {astar_time:<15.4f} {dijkstra_time:<15.4f} {time_winner:<10}")
        
        # Path cost
        astar_cost = astar_result['total_cost']
        dijkstra_cost = dijkstra_result['total_cost']
        cost_winner = "A*" if astar_cost < dijkstra_cost else "Dijkstra" if dijkstra_cost < astar_cost else "Tie"
        print(f"{'Path Cost':<25} {astar_cost:<15.2f} {dijkstra_cost:<15.2f} {cost_winner:<10}")
        
        # Path length
        astar_length = astar_result['path_length']
        dijkstra_length = dijkstra_result['path_length']
        length_winner = "A*" if astar_length < dijkstra_length else "Dijkstra" if dijkstra_length < astar_length else "Tie"
        print(f"{'Path Length':<25} {astar_length:<15} {dijkstra_length:<15} {length_winner:<10}")
        
        print("-" * 60)
        
        # Efficiency analysis
        if astar_nodes > 0 and dijkstra_nodes > 0:
            efficiency_ratio = dijkstra_nodes / astar_nodes
            print(f"\nEFFICIENCY ANALYSIS:")
            print(f"A* explored {efficiency_ratio:.2f}x fewer nodes than Dijkstra")
            if astar_time > 0:
                print(f"A* was {dijkstra_time/astar_time:.2f}x faster than Dijkstra")
            else:
                print("A* time is too small to compare speed ratio.")
        
        # Memory usage estimation
        astar_memory = astar_nodes * 3 * 8  # 3 floats per state, 8 bytes per float
        dijkstra_memory = dijkstra_nodes * 3 * 8
        print(f"\nESTIMATED MEMORY USAGE:")
        print(f"A*: ~{astar_memory/1024:.1f} KB")
        print(f"Dijkstra: ~{dijkstra_memory/1024:.1f} KB")
        
        # Prominently print path lengths
        print(f"\nA* Path Length: {astar_result['path_length']} nodes")
        print(f"Dijkstra Path Length: {dijkstra_result['path_length']} nodes")

def get_user_input():
    """Get user input for algorithm parameters"""
    print("Path Planning: A* vs Dijkstra Comparison")
    print("=" * 50)
    
    try:
        # Get clearance and robot radius
        clearance = float(input("Enter Obstacle Clearance: "))
        robot_radius = float(input("Enter Robot Radius: "))
        step_size = float(input("Enter Robot Step Size: "))
        
        # Get map dimensions
        map_width = int(input("Enter Map Width: "))
        map_height = int(input("Enter Map Height: "))
        
        # Get start position
        start_x = float(input("Enter Start X coordinate: "))
        start_y = float(input("Enter Start Y coordinate: "))
        start_theta = float(input("Enter Initial Theta (degrees): "))
        
        # Get goal position
        goal_x = float(input("Enter Goal X coordinate: "))
        goal_y = float(input("Enter Goal Y coordinate: "))
        goal_theta = float(input("Enter Final Theta (degrees): "))
        
        start = (start_x, start_y, start_theta)
        goal = (goal_x, goal_y, goal_theta)
        
        return clearance, robot_radius, step_size, map_width, map_height, start, goal
        
    except ValueError as e:
        print(f"Error: Invalid input! Please enter numeric values only.")
        print(f"Details: {e}")
        return None
    except EOFError:
        print("Error: Unexpected end of input.")
        return None

def main():
    """Main function"""
    try:
        # Get user input
        user_input = get_user_input()
        
        if user_input is None:
            print("Invalid input provided. Exiting...")
            return
            
        clearance, robot_radius, step_size, map_width, map_height, start, goal = user_input
        
        print(f"\nInput Summary:")
        print(f"Clearance: {clearance}, Robot Radius: {robot_radius}, Step Size: {step_size}")
        print(f"Map Dimensions: {map_width}x{map_height}")
        print(f"Start: ({start[0]}, {start[1]}, {start[2]}°)")
        print(f"Goal: ({goal[0]}, {goal[1]}, {goal[2]}°)")
        print()
        
        # Create path planner
        planner = PathPlanner(clearance, robot_radius, step_size, map_width, map_height)
        
        # Validate start and goal positions
        if not planner.is_valid_point(start[0], start[1]):
            print("Error: Start position is in obstacle space!")
            print("Please try with different start coordinates.")
            return
        
        if not planner.is_valid_point(goal[0], goal[1]):
            print("Error: Goal position is in obstacle space!")
            print("Please try with different goal coordinates.")
            return
        
        print("✓ Both start and goal positions are valid!")
        print("Starting algorithm comparison...\n")
        
        # Run comparison
        astar_result, dijkstra_result = planner.compare_algorithms(start, goal)
        
        # Show visualization
        plt.show()
        
        # Save results (optional)
        print(f"\nComparison completed successfully!")
        
    except KeyboardInterrupt:
        print("\nProgram interrupted by user.")
    except ValueError as e:
        print(f"Input Error: Please enter valid numeric values.")
        print(f"Error details: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        print("Please check your input values and try again.")

if __name__ == "__main__":
    # Interactive mode - get user input
    main()