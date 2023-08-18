#include "../include/utils.hpp"



/**
 * returns true if the drone's position is equal (within predefined tolerance) to the desired waypoint position 
*/
bool drone_is_approximately_at_search_waypoint(const mavros_msgs::PositionTarget waypoint, 
                                        const geometry_msgs::PoseStamped current_pose,
                                        const double position_component_tolerance) 
{
    // extract position values from PositionTarget msgs and PoseStamped msgs 
    double x_inertial_desired = waypoint.position.x;
    double y_inertial_desired = waypoint.position.y; 
    double z_inertial_desired = waypoint.position.z; 


    double x_inertial_current = current_pose.pose.position.x;
    double y_inertial_current = current_pose.pose.position.y;
    double z_inertial_current = current_pose.pose.position.z;

    // if the absolute value between the positions of the drone in the inertial frame and the desired waypoint in the inertial 
    // frame are all less than some tolerance, then that means the drone is approximately at the reference point, so return true
    bool x_within_tolerance = std::abs(x_inertial_desired - x_inertial_current) < position_component_tolerance;
    bool y_within_tolerance = std::abs(y_inertial_desired - y_inertial_current) < position_component_tolerance;
    bool z_within_tolerance = std::abs(z_inertial_desired - z_inertial_current) < position_component_tolerance;

    // only return true if all of the abolute values are within the tolerances
    // WE CAN PLAY WITH if we want to change drone search speed 
    // I suggest to OR x,y with z because it's more important to maintain x,y 
    // to follow the path, compared with puttin a hard constraint on the altitude 
    return x_within_tolerance && y_within_tolerance && z_within_tolerance;
}



std::vector<mavros_msgs::PositionTarget> waypoints;
/**
 * brief Generates waypoints for a lawnmower-style search pattern.
 * 
 * param length Length of the search area. (ALONG Y AXIS OR NORTH for ENU frame)
 * param width Width of the search area. (ALONG X AXIS OR EAST for ENU frame)
 * param altitude Fixed Altitude at which the drone should fly.
 * param interval Number of intervals between parallel lines. 
 * param search_time_sec Time to cover the search area, in seconds.
 * return void
 */

void generate_search_waypoints(double length, double width, double altitude, int interval, int search_time_sec){
    // Define the number of steps
    int num_of_steps = RATE*search_time_sec;

    // Define the area limits here
    double x0 = 0.0, y0 = 0.0, x_W = x0 + width, y_L = y0 + length;

    // Fixed altitude
    double z = altitude;

    // Step size along x axis (EAST)
    double step_size = (interval+1)*width / num_of_steps;

    // Boolean to toggle direction
    bool forward = true;

    for (double y = y0; y <= y_L; y += length/interval){
        mavros_msgs::PositionTarget waypoint;
        waypoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        
        waypoint.type_mask = mavros_msgs::PositionTarget::IGNORE_VX + mavros_msgs::PositionTarget::IGNORE_VY +
                             mavros_msgs::PositionTarget::IGNORE_VZ + mavros_msgs::PositionTarget::IGNORE_AFX +
                             mavros_msgs::PositionTarget::IGNORE_AFY + mavros_msgs::PositionTarget::IGNORE_AFZ +
                             mavros_msgs::PositionTarget::FORCE + mavros_msgs::PositionTarget::IGNORE_YAW +
                             mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
        waypoint.yaw = PI/2;
        waypoint.position.z = z;
        waypoint.position.y = y;

        // Generate waypoints
        if (forward){
            for (double x = x0; x <= x_W; x += step_size){
                waypoint.position.x = x;
                waypoints.push_back(waypoint);
            }
        } else {
            for (double x = x_W; x >= x0; x -= step_size){
                waypoint.position.x = x;
                waypoints.push_back(waypoint);
            }
        }
        
        // Switch direction
        forward = !forward;
    }
}

std::vector<mavros_msgs::PositionTarget> get_search_waypoints() {
    return waypoints;
}

