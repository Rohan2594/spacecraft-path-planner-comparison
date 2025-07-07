# 🚀 Spacecraft Path Planner Comparison

A web-based simulation tool that compares **A\*** and **Dijkstra’s Algorithm** for autonomous spacecraft path planning in a 2D grid-based environment. This project enables users to input mission-specific parameters, visualize the resulting paths, and analyze the performance metrics of both algorithms.

---

## 🌌 Features

- Compare **A\*** vs **Dijkstra's Algorithm** in a spacecraft navigation context
- Input and customize:
  - Obstacle clearance
  - Robot radius
  - Step size
  - Map dimensions
  - Start and goal coordinates + orientation
- Real-time 2D grid visualization with:
  - Obstacles
  - Explored nodes
  - Planned path
  - Start & Goal markers
- Performance metrics comparison:
  - Nodes explored
  - Time taken
  - Path cost
  - Path length
- Flask-based web interface

---

## 📸 Screenshots

![Figure_1](https://github.com/user-attachments/assets/222526fb-cd98-4b51-aa0c-d0408404850e)
![WhatsApp Image 2025-06-30 at 10 08 05_69760c1b](https://github.com/user-attachments/assets/4e4efbd6-87c0-4a85-9dda-f6380d97f062)



## 🔧 Installation

1. **Clone the repository**
   ```bash
   git clone https://github.com/yourusername/spacecraft-path-planner-comparison.git
   cd spacecraft-path-planner-comparison
Create a virtual environment
bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
Install dependencies

bash
pip install -r requirements.txt
🚀 Running the App
1. Web Interface (Flask)
bash
python app.py
Then open your browser and go to:
📍 http://localhost:5000

2. CLI Mode (Manual Input)
bash
python main.py
Follow the prompts in the terminal to provide map and robot parameters.

📁 Project Structure
php
Copy
Edit
├── app.py                  # Flask web interface
├── main.py                 # Core algorithm logic and CLI
├── requirements.txt        # Required Python packages
├── static/                 # Saved output images
├── templates/
│   └── index.html          # HTML form UI
📊 Algorithms Used
A* Search
Uses a heuristic (Euclidean distance) to guide exploration

Explores fewer nodes in most cases

Generally faster and more efficient

Dijkstra’s Algorithm
Uniform-cost search (no heuristic)

Explores all possible shortest paths

Often slower but guaranteed to find optimal path

📦 Dependencies
Flask==2.3.3

NumPy==1.24.3

Matplotlib==3.7.2

Werkzeug==2.3.7

Install via:

bash
pip install -r requirements.txt
