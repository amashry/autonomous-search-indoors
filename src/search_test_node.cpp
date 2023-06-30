#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/PositionTarget.h>

#define PI  3.14159265358979323846264338327950
#define RATE            30  // loop rate hz
const double POSITION_COMPONENT_TOLERANCE_M = 0.10; // 10 centimeters, but represented in units of meters


std::vector<mavros_msgs::PositionTarget> waypoints;


void generate_waypoints(){
    double LENGHT = 4.5, WIDTH = 3.5; // Define Area Size
    int INTERVAL = 4; // Define number of intervals between parallel lines
    int SEARCH_TIME_SEC = 30; // Define time to cover the area in seconds 
    
    int num_of_steps = RATE*SEARCH_TIME_SEC;

    double x0 = 0.0, y0 = 0.0, x_W = x0 + WIDTH, y_L = y0 + LENGHT; // Define the area limits here
    double z = 1.0; // Fixed altitude
    // double step_size = (INTERVAL+1)*WIDTH / num_of_steps; // Step size along x axis
    double step_size = 1.0;
    

    bool forward = true; // Start moving in positive x direction
    // double y1 = y0; 

    for (double y = y0; y <= y_L; y += LENGHT/INTERVAL){
        mavros_msgs::PositionTarget waypoint;
        waypoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        waypoint.type_mask = mavros_msgs::PositionTarget::IGNORE_VX + mavros_msgs::PositionTarget::IGNORE_VY +
                             mavros_msgs::PositionTarget::IGNORE_VZ + mavros_msgs::PositionTarget::IGNORE_AFX +
                             mavros_msgs::PositionTarget::IGNORE_AFY + mavros_msgs::PositionTarget::IGNORE_AFZ +
                             mavros_msgs::PositionTarget::FORCE + mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
        waypoint.yaw = PI/2;
        waypoint.position.z = z;
        waypoint.position.y = y;
        if (forward){
            for (double x = x0; x <= x_W; x += step_size){
                waypoint.position.x = x;
                waypoints.push_back(waypoint);
            }
        }
        else{
            for (double x = x_W; x >= x0; x -= step_size){
                waypoint.position.x = x;
                waypoints.push_back(waypoint);
            }
        }
        forward = !forward; // Switch direction
    }
}

/**
 * returns true if the drone's position is equal (within some tolerance) to the desired waypoint position 
*/
bool drone_is_approximately_at_waypoint(const mavros_msgs::PositionTarget waypoint, 
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
    return x_within_tolerance && y_within_tolerance && z_within_tolerance;
}

geometry_msgs::PoseStamped current_pose;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "search_test");
    ros::NodeHandle nh;

    ros::Publisher local_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
    ros::Subscriber current_pos = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, pose_cb);
    
    generate_waypoints();

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
        for(int i = 0; i < waypoints.size();){
            local_pos_pub.publish(waypoints[i]);
            ROS_INFO_STREAM("Current Waypoint is = \n" << waypoints[i]);
            if (drone_is_approximately_at_waypoint(waypoints[i],current_pose,POSITION_COMPONENT_TOLERANCE_M))
            {
                ROS_INFO_STREAM("CURRENTLY AT WAYPOINT!");
                i++;
            }
            ros::spinOnce();
            rate.sleep();
        }
    }


    return 0;
}
