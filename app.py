from flask import Flask, render_template, request, send_file
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend for web server
import matplotlib.pyplot as plt
import os
from main import PathPlanner

app = Flask(__name__)

@app.route('/', methods=['GET', 'POST'])
def index():
    result_img = None
    error = None
    stats = None
    if request.method == 'POST':
        try:
            # Extract form parameters
            clearance = int(request.form['clearance'])
            robot_radius = int(request.form['robot_radius'])
            step_size = int(request.form['step_size'])
            map_width = int(request.form['map_width'])
            map_height = int(request.form['map_height'])
            start_x = int(request.form['start_x'])
            start_y = int(request.form['start_y'])
            start_theta = int(request.form['start_theta'])
            goal_x = int(request.form['goal_x'])
            goal_y = int(request.form['goal_y'])
            goal_theta = int(request.form['goal_theta'])

            # Create planner with all parameters
            planner = PathPlanner(clearance, robot_radius, step_size, map_width, map_height)
            start = (start_x, start_y, start_theta)
            goal = (goal_x, goal_y, goal_theta)
            
            # Validate start and goal positions
            if not planner.is_valid_point(start_x, start_y):
                error = "Start position is in obstacle space! Please choose a different start point."
            elif not planner.is_valid_point(goal_x, goal_y):
                error = "Goal position is in obstacle space! Please choose a different goal point."
            else:
                # Run algorithm comparison
                astar_result, dijkstra_result = planner.compare_algorithms(start, goal)
                
                # Check if paths were found
                if astar_result['path'] is None and dijkstra_result['path'] is None:
                    error = "No path found by either algorithm! Try different start/goal positions or adjust robot parameters."
                else:
                    # Create static directory if it doesn't exist
                    if not os.path.exists('static'):
                        os.makedirs('static')
                    
                    # Save the comparison graph
                    plt.savefig('static/comparison.png', dpi=150, bbox_inches='tight')
                    plt.close('all')  # Close all figures to prevent memory leaks
                    
                    result_img = 'comparison.png'
                    stats = {
                        'astar': astar_result,
                        'dijkstra': dijkstra_result
                    }
                    
        except ValueError as e:
            error = f"Invalid input values: {e}"
        except Exception as e:
            error = f"An error occurred: {e}"
            print(f"Error in path planning: {e}")  # Log error for debugging
    
    return render_template('index.html', result_img=result_img, error=error, stats=stats)

@app.route('/static/<filename>')
def static_files(filename):
    """Serve static files (images)"""
    try:
        return send_file(f'static/{filename}', mimetype='image/png')
    except Exception as e:
        print(f"Error serving static file {filename}: {e}")
        return "File not found", 404

if __name__ == '__main__':
    print("Starting Path Planning Web Application...")
    print("Open your browser and go to: http://localhost:5000")
    app.run(debug=True, host='0.0.0.0', port=5000)