#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/PositionTarget.h>

#include "../include/utils.hpp"


#define PI  3.14159265358979323846264338327950
#define RATE            20  // loop rate hz ---> ADD THIS TO MAIN PROJECT 
const double POSITION_COMPONENT_TOLERANCE_M = 0.40; // 40 centimeters (0.4m)


std::vector<mavros_msgs::PositionTarget> waypoints;


geometry_msgs::PoseStamped current_pose;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
}

// global variable to keep track of the current search waypoint index
int current_wp_index = 0;

// Arbitrary condition to trigger custom behavior
// In this example, the custom behavior is triggered every 50 waypoints
const int CONDITION_WAYPOINT_INTERVAL = 50;

// bool variable to check if we finished search
bool FINISHED_SEARCHING{false};

int main(int argc, char **argv){
    ros::init(argc, argv, "search_test");
    ros::NodeHandle nh;

    ros::Publisher local_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
    ros::Subscriber current_pos = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, pose_cb);
    
    // Define the desired search params for waypoint generation function
    double length = 4.5, width = 3.5;
    double altitude = 1.5;
    int interval = 3;
    int search_time_sec = 10;
    
    generate_search_waypoints(length, width, altitude, interval, search_time_sec);
    // Use the getter function to access the waypoints
    waypoints = get_search_waypoints();

    ros::Rate rate(RATE);
    // send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(waypoints[0]);
        ROS_INFO_STREAM(waypoints[0]);
        ros::spinOnce();
        rate.sleep();
    }
    std::cout << "ROS STARTED";


    while(ros::ok()){
        while (!drone_is_approximately_at_search_waypoint(waypoints[current_wp_index],current_pose,POSITION_COMPONENT_TOLERANCE_M)){
            // keep publishing the waypoint until you get at the waypoint 
            local_pos_pub.publish(waypoints[current_wp_index]);
            ROS_INFO_STREAM("Current Waypoint is = \n" << waypoints[current_wp_index]);
            ROS_INFO_STREAM("CURRENTLY AT WAYPOINT: " << current_wp_index);
            ros::spinOnce();
            rate.sleep();
        }

        // if current_wp_index reaches the end of waypoints, reset it to home
        if (current_wp_index >= waypoints.size()-1){
            current_wp_index = 0;
            FINISHED_SEARCHING = true; 
        }

        // If arbitrary condition is met, execute custom behavior
        if (current_wp_index % CONDITION_WAYPOINT_INTERVAL == 0 && !FINISHED_SEARCHING && current_wp_index != 0){
            
            // Simple custom behavior: yaw 90 degrees to the right and then return
            mavros_msgs::PositionTarget custom_wp = waypoints[current_wp_index];
            custom_wp.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

            custom_wp.type_mask = mavros_msgs::PositionTarget::IGNORE_VX + mavros_msgs::PositionTarget::IGNORE_VY +
            mavros_msgs::PositionTarget::IGNORE_VZ + mavros_msgs::PositionTarget::IGNORE_AFX +
            mavros_msgs::PositionTarget::IGNORE_AFY + mavros_msgs::PositionTarget::IGNORE_AFZ +
            mavros_msgs::PositionTarget::FORCE + mavros_msgs::PositionTarget::IGNORE_YAW +
            mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

            custom_wp.yaw = 3*PI/4;
            custom_wp.position.z = 2; 
            for (int i = 0; i < 5*RATE; ++i){  // yaw for 5 second
                local_pos_pub.publish(custom_wp);
                ROS_INFO_STREAM("CONDITION MET, EXECUTING CUSTOM BEHAVIOR");
                ROS_INFO_STREAM("custom behaviour Waypoint is = \n" << custom_wp);
                ros::spinOnce();
                rate.sleep();
            }

            for (int i = 0; i < 3*RATE; ++i){  // return to original yaw angle over 1 second
                local_pos_pub.publish(waypoints[current_wp_index]);
            }
            ROS_INFO_STREAM("CUSTOM BEHAVIOR COMPLETED");

        }
        if (!FINISHED_SEARCHING)
        {
            current_wp_index++;
        }
        while (FINISHED_SEARCHING)
        {
            // keep hovering at home position 
            local_pos_pub.publish(waypoints[current_wp_index]);
            ROS_INFO_STREAM("Home position waypoint = \n" << waypoints[current_wp_index]);
            ROS_INFO_STREAM("Returned Home: " << current_wp_index);
            ros::spinOnce();
            rate.sleep();
        }

    }


    return 0;
}