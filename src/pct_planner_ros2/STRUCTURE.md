# PCT Planner ROS2 Package Structure

## Directory Structure
```
pct_planner_ros2/
├── package.xml              # ROS2 package manifest
├── setup.py                 # Python package setup
├── README.md               # Package documentation
├── MIGRATION_SUMMARY.md    # Migration details
├── launch/
│   └── pct_planner.launch.py  # Launch file
├── pct_planner_ros2/       # Main Python package
│   ├── __init__.py         # Package init
│   ├── plan.py             # Main ROS2 node
│   ├── config.py           # Configuration
│   ├── utils.py            # Utility functions
│   ├── planner_wrapper.py  # Planner interface
│   └── tomography.py       # Tomography node (template)
└── test/                   # Test scripts
    └── test_pct_planner.py # Test script
```

## Key Components

### Main Node (plan.py)
- PCTPlannerNode: Main ROS2 node implementation
- Handles path planning and publishing
- Subscribes to goal topics
- Publishes Path messages to /pct_path

### Configuration (config.py)
- ConfigPlanner: Planner-specific settings
- ConfigWrapper: Wrapper-specific settings
- Config: Main configuration class

### Utilities (utils.py)
- traj2ros: Convert trajectory to ROS Path message
- Handles message formatting for ROS2 compatibility

### Planner Interface (planner_wrapper.py)
- TomogramPlanner: Main planner interface
- Handles tomogram loading and path planning
- Provides mock implementation for ROS2 compatibility

## ROS2 Interface

### Published Topics
- `/pct_path` (nav_msgs/Path): Planned trajectory

### Subscribed Topics  
- `/goal` (geometry_msgs/PoseStamped): Goal position
- `/move_base_simple/goal` (geometry_msgs/PoseStamped): RViz2 goals

### Parameters
- `scene` (string): Scene name (default: 'Spiral')
- `publish_rate` (int): Publishing rate (default: 10)

## Dependencies
- rclpy: ROS2 Python client
- std_msgs: Standard messages
- geometry_msgs: Geometry messages  
- nav_msgs: Navigation messages
- visualization_msgs: Visualization messages
- Python libraries: numpy, cupy, open3d