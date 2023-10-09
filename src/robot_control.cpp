#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <sstream>

#define PI 3.1415926

// Global Variabies Definition
sensor_msgs::LaserScan laser_data;
geometry_msgs::Twist cmd_vel;
bool manual_control = false;
bool low_distance = false;

// Callback Function Definition
void vel_callback(const geometry_msgs::Twist::ConstPtr &msgs) {
    manual_control = true;
    cmd_vel = *msgs;
    ROS_INFO("Manual Command: Forward velocity %2.2f; Angular velocity %2.2f", msgs->linear.x, msgs->angular.z);
}

void laser_callback(const sensor_msgs::LaserScan::ConstPtr &msgs) {
    laser_data = *msgs;
}

int main(int argc, char **argv) {
    // Init the node
    ros::init(argc, argv, "robot_no_crash_node");

    // Init the handle
    ros::NodeHandle node_handle;

    // Init the subscriber
    ros::Subscriber sub_1 = node_handle.subscribe("des_vel", 10, vel_callback);
    ros::Subscriber sub_2 = node_handle.subscribe("laser_1", 10, laser_callback);

    // Init the publisher
    ros::Publisher cmd_vel_pub = node_handle.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

    // Init the loop_rate
    ros::Rate loop_rate(10);
    
    
    // Init velocity
    cmd_vel.linear.x = 0.1;
    cmd_vel.linear.y = 0;
    cmd_vel.linear.z = 0;
    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    cmd_vel.angular.z = 0;
    
    // Init the wall_dist
    double wall_dist = 1;
    ROS_INFO_ONCE("wall_dist began with: [%2.2f]", wall_dist);
    if (node_handle.getParamCached("/wall_dist", wall_dist)) {
        ROS_INFO("wall_dist is updated to: [%2.2f]", wall_dist);
    } else {
        ROS_ERROR("Failed to get param 'wall_dist'");
    }
    ROS_INFO_ONCE("wall_dist is now: [%2.2f]", wall_dist);

    // Init detect angle
    float detect_angle = 0.5 * PI;

    // Run the robot
    while (ros::ok()) {
        // Detect the distance
        low_distance = false;
        int range_length = ceil((laser_data.angle_max - laser_data.angle_min) / laser_data.angle_increment);
        for (int i = 0; i < range_length; i++) {
            float scan_angle = laser_data.angle_min + i * laser_data.angle_increment;
            if (scan_angle >= -detect_angle && scan_angle <= detect_angle) {
                if (laser_data.ranges[i] <= wall_dist) {
                    low_distance = true;
                    break;
                }
            }
        }

        // determine what to publish and publish the cmd
        if (range_length > 0) {
            if (low_distance) {
                cmd_vel.linear.x = 0;
                cmd_vel.angular.z = 0.2;
                ROS_WARN("Auto control: Turn");
                manual_control = false;
            } else {
                if (!manual_control) {
                    cmd_vel.linear.x = 0.1;
                    cmd_vel.angular.z = 0;
                    ROS_INFO("Auto control: Move");
                } else {
                    ROS_INFO("Manual control");
                    manual_control = false;
                }
            }
            cmd_vel_pub.publish(cmd_vel);
        } else {
            ROS_WARN("No connected robot");
        }
        
        // Update
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
