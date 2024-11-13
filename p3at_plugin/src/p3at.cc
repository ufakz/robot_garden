#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"
#include <vector>

#include "get_flower_locations.h"

ros::Publisher cmd_vel_pub;
std::vector<std::pair<float, float>> waypoints;
size_t current_waypoint = 0;
double robot_x = 0.0;
double robot_y = 0.0;
double robot_yaw = 0.0;
bool pose_initialized = false;

const double DISTANCE_THRESHOLD = 0.2;
const double ANGULAR_THRESHOLD = 0.1;
const double LINEAR_SPEED = 0.3;
const double ANGULAR_SPEED = 0.5;

void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    robot_x = msg->pose.pose.position.x;
    robot_y = msg->pose.pose.position.y;
    
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, robot_yaw);
    
    pose_initialized = true;
}

double getDistanceToGoal(double goal_x, double goal_y)
{
    return std::sqrt(std::pow(goal_x - robot_x, 2) + 
                    std::pow(goal_y - robot_y, 2));
}

double getAngleToGoal(double goal_x, double goal_y)
{
    double angle = std::atan2(goal_y - robot_y, goal_x - robot_x) - robot_yaw;
    return std::atan2(std::sin(angle), std::cos(angle));
}

bool moveToNextWaypoint(geometry_msgs::Twist &cmd_vel)
{
    if (current_waypoint >= waypoints.size()) {
        return true;
    }

    double goal_x = waypoints[current_waypoint].first;
    double goal_y = waypoints[current_waypoint].second;
    
    double distance = getDistanceToGoal(goal_x, goal_y);
    double angle = getAngleToGoal(goal_x, goal_y);

    if (std::abs(angle) > ANGULAR_THRESHOLD) {
        cmd_vel.angular.z = angle > 0 ? ANGULAR_SPEED : -ANGULAR_SPEED;
        cmd_vel.linear.x = 0;
    }
    else if (distance > DISTANCE_THRESHOLD) {
        cmd_vel.linear.x = LINEAR_SPEED;

        // Some angular speed here?
        cmd_vel.angular.z = 0.2 * angle;
    }
    else {
        current_waypoint++;
        ROS_INFO("Reached waypoint %lu", current_waypoint);
        return false;
    }

    return false;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "p3at");
    ros::NodeHandle nh;

    // Get flower locations using the function
    std::string world_file;
    nh.param<std::string>("world_file", world_file, "garden.world");
    waypoints = getFlowerPotLocations(world_file);
    ROS_INFO("Loaded %lu waypoints", waypoints.size());

    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    
    ros::Subscriber laser_sub = nh.subscribe("/hokuyo", 1000, laserCallback);
    ros::Subscriber odom_sub = nh.subscribe("/odom", 1000, odomCallback);

    ros::Rate rate(20);

    // Get the first pose which is the initial location of the robot
    while(ros::ok() && !pose_initialized) {
        ros::spinOnce();
        rate.sleep();
    }

    while (ros::ok())
    {
        geometry_msgs::Twist msg;
        
        if (moveToNextWaypoint(msg)) {
            ROS_INFO("All waypoints reached!");
            break;
        }
        
        cmd_vel_pub.publish(msg);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}