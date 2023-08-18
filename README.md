
![lawnmower_search_fig](https://github.com/amashry/autonomous-search-indoors/assets/98168605/7b07b0ef-f52c-413b-99da-336e6f83761c)

# Simulation 
![search_grid_simulation](https://github.com/amashry/autonomous-search-indoors/assets/98168605/630dc660-cc9f-40da-94bc-119e8b76bea8)

full video: https://drive.google.com/file/d/1LT0K3gaqwdd0eGpYQzA2Q_Wcw3-fH4S5/view?usp=sharing 

# Real Flight Test

https://github.com/amashry/autonomous-search-indoors/assets/98168605/9f4a26d6-25d0-4ddd-b296-d58986dcae58

# autonomous-search-indoors
basic search algorithm for autonomous indoors missions using PX4 and MAVROS

This ROS node is a basic implementation of left-to-right lawnmower search algorithm using MAVROS and PX4. Tested in Gazebo simulation and on a real vehicle (ModalAI m500 drone). 

The current implementation has been integrated into another mission, however, it has been added here to generalize for several other applications. The implementation generates search waypoints for the drone to follow, with user-defined parameters for the desired search area, altitude, and time. Then, it shows an example for taking an action once some arbitrary condition is met while searching. 

The current custom action that’s integrated is for the drone to stop and climb to a higher altitude for a short time, and then goes back to its last published search waypoint. 

The figure shows how `generate_waypoints_function` works. Given that the drone starts at the `(0,0)` position (home position) inside the desired area, and facing straight ahead (aligning with the ENU frame). The function takes the following as inputs: 

- The boundary size of the area `width` (along X), and `length` (along Y) in `meters` .
- Number of overlap intervals across the Y axis (as shown in the figure)
- Desired altitude in meters for the drone to fly at.
- Search time in seconds, which changes the speed of the drone to cover the desired area.

 

Using ROS standard coordinate frames, so the local inertial frame is in ENU (X East, Y North, Z Up). MAVROS then publishes the desired waypoints as PositionTarget messages to PX4 as setpoints. Since PX4’s native coordinate system is in NED (X **N**orth, Y **E**ast, Z **D**own), we have to specify `coordinate_frame` in the `mavros_msgs::PositionTarget` to be `FRAME_LOCAL_NED`.
## TODO 
1. add documentation for this (illustration, simulation vid and actual mission)
2. make the code more modular to be easily integrated later in different programs.
3. integrate it into the main autonomous program
4. Update the code and commit the current implementation
5. For documentation, make sure to add how to clone the code and catkin_make it and even include the steps for how to create a catkin_ws if you don't have one. This will help the code be easily re-usable.
6. Add another layer for the search where you integrate the april tag in simulation and search for it.
7. Also, add the april tag tracking simulation and actual tracking. Test it in the lab
