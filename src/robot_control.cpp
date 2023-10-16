#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

void laserScan_callback(const sensor_msgs::LaserScan::ConstPtr &msgs);
void desvel_callback(const geometry_msgs::Twist::ConstPtr &msgs);

// Globals
bool hit = false;
sensor_msgs::LaserScan laser_data;
geometry_msgs::Twist cmd_vel;


// Callback
void laserScan_callback(const sensor_msgs::LaserScan::ConstPtr &msgs) {
    laser_data = *msgs;
}

void desvel_callback(const geometry_msgs::Twist::ConstPtr &msgs) {
    cmd_vel = *msgs;
    ROS_INFO("Manual Command: Forward velocity %2.2f; Angular velocity %2.2f", msgs->linear.x, msgs->angular.z);
}

int main(int argc, char **argv) {
    // Init node, nodehandle, publisher and subscriber, loop rate
    ros::init(argc, argv, "robot_no_crash_node");
    ros::NodeHandle publisher_handle;
    ros::Publisher p_pub = publisher_handle.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    ros::Subscriber sub_vel = publisher_handle.subscribe("des_vel", 10, desvel_callback);
    ros::Subscriber sub_laser = publisher_handle.subscribe("laser_1", 10, laserScan_callback);
    ros::Rate loop_rate(10);
    
    // Initial speed
    cmd_vel.linear.x = 0.2;
    cmd_vel.linear.y = 0;
    cmd_vel.linear.z = 0;
    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    cmd_vel.angular.z = 0;
    
    // Init wall_dist with a default value
    double wall_dist = 1.0;
    // control the sensitivity of sensors when detecting hit
    double detect_sensitivity  = 45.0;
    ROS_INFO_ONCE("wall_dist began with: [%2.2f]", wall_dist);
    
    if (publisher_handle.getParamCached("/wall_dist", wall_dist)) {
        ROS_INFO("wall_dist was updated to: [%2.2f]", wall_dist);
    }
    
    ROS_INFO_ONCE("wall_dist is now: [%2.2f]", wall_dist);

    // Detect angle of the laser, consider data within [-1/2*PI, 1/2*PI] to avoid collision
    const double detect_angle = detect_sensitivity * 3.14 / 180;
    
    while (ros::ok()) {
        hit = false;
        float current_discance = 0.0;
        int detect_range = (laser_data.angle_max - laser_data.angle_min) / laser_data.angle_increment;
        // Only data within detect angle will be taken into consideration
        for (int i = 0; i < detect_range; i++) {
            float scan_angle = laser_data.angle_min + laser_data.angle_increment * i;
            if (scan_angle >= -detect_angle && scan_angle <= detect_angle) {
                if (laser_data.ranges[i] <= wall_dist) { // the distance at a certain angle
                    current_discance = laser_data.ranges[i];
                    hit = true;
                    break;
                }
            }
        }
       
        if (detect_range > 0) {
            if (hit) {
                cmd_vel.linear.x = 0;
                cmd_vel.angular.z = 0.2;            
                ROS_WARN("Turning");
            } 
            else {
            	cmd_vel.linear.x = 0.2;
            	cmd_vel.angular.z = 0;
           	ROS_INFO("Moving Forward");
            }
            p_pub.publish(cmd_vel);
        } 
        else {
            ROS_WARN("No Robot Connected!");
        }
        

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
