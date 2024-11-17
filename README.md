# UAV Path Planning with Grey Wolf Optimizer

A Python implementation of 3D path planning for Unmanned Aerial Vehicles (UAVs) using the Grey Wolf Optimizer (GWO) algorithm. This project provides robust path optimization with obstacle avoidance capabilities and stunning 3D visualizations of the optimization process.

![Path Planning Animation](path_animation.gif)

## Features

- **Intelligent Path Planning**: Implements Grey Wolf Optimizer for finding optimal UAV paths
- **3D Collision Avoidance**: Handles multiple cylindrical no-fly zones in 3D space
- **Real-time Visualization**: 
  - Dynamic 3D visualization of the optimization process
  - Color-coded path quality indicators
  - Interactive camera rotation
  - Progress tracking with fitness metrics
- **Customizable Parameters**:
  - Adjustable UAV constraints
  - Configurable no-fly zones
  - Tunable optimization parameters
- **Data Export/Import**: Save and load optimization results for further analysis
- **Reproducible Results**: Seed management for consistent outcomes

## Requirements

- Python 3.8+
- NumPy
- Matplotlib
- tqdm
- FFmpeg (for animation export)

## Installation

1. Clone the repository:
```bash
git clone https://github.com/ChenDavidTimothy/gwo-path-planner.git
cd gwo-path-planner
```

2. Install the required Python packages:
```bash
pip install numpy matplotlib tqdm
```

3. Install FFmpeg (required for animation export):
```bash
# Windows (using chocolatey)
choco install ffmpeg
```

## Project Structure

```
├── src/
│   ├── core/                 # Core algorithm implementations
│   │   ├── gwo.py           # Grey Wolf Optimizer algorithm
│   │   ├── obj_fun.py       # Objective function for path evaluation
│   │   └── uav_setup.py     # UAV configuration and constraints
│   │
│   ├── utils/               # Utility functions
│   │   ├── encoders.py      # JSON encoders for NumPy arrays
│   │   └── export.py        # Data export functionality
│   │
│   └── visualization/       # Visualization tools
│       ├── animation.py     # 3D animation creation
│       ├── config.py        # Visualization settings
│       └── utils.py         # Visualization utilities
│
└── main.py                  # Main execution file
```

## Usage

1. Basic execution:
```python
python main.py
```

2. Configure UAV parameters in `src/core/uav_setup.py`:
```python
UAV = {
    'S': np.array([2, 2, 15]),  # Start position
    'G': np.array([18, 18, 8]), # Goal position
    'PointNum': 3,              # Number of navigation points
    'NoFlyZones': [...],        # No-fly zone definitions
    'limt': {...}               # Spatial constraints
}
```

3. Adjust optimization parameters in `main.py`:
```python
SearchAgents = 30  # Number of search agents
Max_iter = 200     # Maximum iterations
seed = 42          # Random seed for reproducibility
```

4. Customize visualization settings in `src/visualization/config.py`:
```python
FPS = 30           # Animation frame rate
DURATION = 60      # Animation duration in seconds
ROTATION_ANGLE = 720  # Camera rotation angle
```

## Algorithm Details

The Grey Wolf Optimizer (GWO) algorithm mimics the social hierarchy and hunting behavior of grey wolves. In this implementation:

1. **Initialization**: Random valid paths are generated
2. **Hierarchy**: The three best solutions are labeled as alpha, beta, and delta
3. **Hunting**: Other solutions (omega wolves) update their positions based on the best solutions
4. **Convergence**: The process continues until optimal paths are found

The objective function considers:
- Total path length
- Collision avoidance with no-fly zones
- UAV movement constraints

## Visualization Features

The 3D visualization includes:
- Start and goal positions
- No-fly zones as transparent cylinders
- Color-coded paths indicating solution quality
- Real-time fitness tracking
- Smooth camera rotation
- Progress indicators

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- Grey Wolf Optimizer algorithm by Seyedali Mirjalili
- Matplotlib for 3D visualization capabilities
- FFmpeg for animation export

## Author

David Timothy - chendavidtimothy@gmail.com
GitHub: [@ChenDavidTimothy](https://github.com/ChenDavidTimothy)