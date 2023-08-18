# Autonomous Indoor Search with PX4 & MAVROS

This ROS node facilitates a left-to-right lawnmower search algorithm integrated with MAVROS and PX4, targeted primarily for autonomous indoor applications.

![Lawnmower Search Illustration](https://github.com/amashry/autonomous-search-indoors/assets/98168605/7b07b0ef-f52c-413b-99da-336e6f83761c)

## Features
- **Versatile Application**: Initially integrated for a specific mission, this implementation is designed to be general enough for various applications.
- **Search Waypoints Generation**: Generates waypoints for the drone to follow based on user-defined parameters, like the search area, altitude, and time duration.
- **Conditional Custom Action**: Demonstrates how to integrate a custom action based on certain conditions during the search. Currently, the drone stops, ascends to a higher altitude briefly, and then resumes from the last search waypoint.

## Simulations and Testing

### Gazebo Simulation
The algorithm has been tested in the Gazebo simulation environment. 

![Search Grid Simulation](https://github.com/amashry/autonomous-search-indoors/assets/98168605/9e1944bf-47f3-49e8-b9ff-0aa4eb79a228)

For a the full video overview, check out the [simulation video](https://drive.google.com/file/d/1LT0K3gaqwdd0eGpYQzA2Q_Wcw3-fH4S5/view?usp=sharing).

### Real Flight Test
The implementation has also been validated using a real drone, particularly the ModalAI m500 drone.

https://github.com/amashry/autonomous-search-indoors/assets/98168605/91bc7c68-134e-46bd-bc28-38f212946184

## How it Works

### Waypoint Generation
Starting from the `(0,0)` position (home position) inside the target area and facing forward (aligned with the ENU frame), the `generate_waypoints_function` computes the waypoints based on the following inputs:
- **Boundary Size**: Defined by `width` (X-axis) and `length` (Y-axis) in meters.
- **Overlap Intervals**: Number of intervals along the Y-axis (as depicted in the figure).
- **Altitude**: Desired flight altitude in meters.
- **Search Time**: Time in seconds, determining the drone speed to cover the specified area.

### Custom Behavior
The current custom behavior serves as a placeholder, showcasing how specific behaviors can be added based on particular conditions. Every 50 waypoints, the drone pauses its search, ascends by half a meter, and hovers for 5 seconds. It then descends back to its original altitude and continues the search pattern. Once the drone completes the entire search grid, it returns to its home position and hovers until manually landed by the user.

### Coordinate Frames
The implementation strictly follows the ROS standard coordinate frames. This means that the local inertial frame is in ENU (East, North, Up). MAVROS communicates the desired waypoints to PX4 as PositionTarget messages or setpoints. Due to PX4's native NED (North, East, Down) coordinate system, it's essential to set the `coordinate_frame` in the `mavros_msgs::PositionTarget` to `FRAME_LOCAL_NED`.

## How to Use

```bash
# Navigate to the 'src' directory of your 'catkin_ws':
cd ~/catkin_ws/src

# Clone the repository:
git clone https://github.com/amashry/autonomous-search-indoors.git

# Go back to the root of your 'catkin_ws' and build the project:
cd ..
catkin_make

# Source the workspace:
source devel/setup.bash

# Launch the node:
roslaunch search_test search_test.launch
```

