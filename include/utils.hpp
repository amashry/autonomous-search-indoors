#ifndef UTILS_HPP
#define UTILS_HPP

#include <ros/ros.h>
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <mavros_msgs/PositionTarget.h>
#include <cmath> // for std::pow

#define PI  3.14159265358979323846264338327950
#define RATE            20


// Function declarations
void generate_search_waypoints(double length, double width, double altitude, int interval, int search_time_sec);
std::vector<mavros_msgs::PositionTarget> get_search_waypoints();
bool drone_is_approximately_at_search_waypoint(const mavros_msgs::PositionTarget waypoint, 
                                        const geometry_msgs::PoseStamped current_pose,
                                        const double position_component_tolerance);

#endif // UTILS_HPP