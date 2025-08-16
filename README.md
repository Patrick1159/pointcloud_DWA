# pointcloud_DWA

A ROS project implementing a Dynamic Window Approach (DWA) local planner that works with point cloud data.

## Project Structure
.
├── build/          # Build artifacts and CMake configuration
├── devel/          # Development space with setup scripts
└── src/            # Source code
    ├── CMakeLists.txt
    └── dwa/        # DWA planner implementation

## Features
- Dynamic Window Approach local planner implementation
- Point cloud data processing
- Receives 2D navigation goals from RViz

## Dependencies
- ROS (Robot Operating System)
- Eigen (for vector operations)
- TF (for coordinate transformations)

## Building the Project

```bash
# Clone the repository
git clone https://github.com/yourusername/pointcloud_DWA.git
cd pointcloud_DWA

# Build the project
catkin_make
```

## Running the Project
1. Source the setup script:   
`source devel/setup.bash`
2. Launch the DWA planner node (replace with your actual launch file):   
`roslaunch dwa dwa_planner.launch`

## Setting Navigation Goals
Navigation goals can be set using RViz's 2D Nav Goal tool.