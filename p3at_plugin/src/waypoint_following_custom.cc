#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/tf.h"
#include <vector>
#include <cmath>
#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <tinyxml2.h>
#include "get_flower_locations.h"
#include "waypoints.h"

class RobotNavigator {
public:
    RobotNavigator();
    void navigate();

private:
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber laser_sub_;

    enum AvoidanceState {
        NONE,
        TURNING,
        MOVING_FORWARD
    };

    AvoidanceState avoidance_state_;
    ros::Time avoidance_start_time_;
    const double TURN_DURATION = 4.0;
    const double FORWARD_DURATION = 5.0;

    std::vector<std::pair<float, float>> flower_pots;
    size_t current_waypoint_;

    double current_x_;
    double current_y_;
    double current_yaw_;
    bool pose_initialized_;
    bool obstacle_detected_;
    bool is_path_clear_;

    int turn_direction;

    std::vector<float> laser_ranges_;

    std::vector<std::pair<float, float>> points;

    const double DISTANCE_THRESHOLD = 0.5;
    const double ANGULAR_THRESHOLD = 0.5;
    const double LINEAR_SPEED = 0.3;
    const double ANGULAR_SPEED = 0.6;
    const double FLOWER_POT_NEAR_RADIUS = 2.0;
    const double OBSTACLE_DISTANCE = 0.5;
    const double CHECK_AHEAD_DISTANCE = 0.2;
    const int FRONT_RAYS = 30;
    const double MAX_TURN_SPEED = 0.5;
    const double MIN_TURN_SPEED = 0.3;
    double obstacle_angle_;

    bool isPathClear();
    bool skipNearbyWaypoint();
    bool isNearFlowerPot(double x, double y);
    int determineOptimalTurnDirection(double obstacle_angle);
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    double getDistanceToGoal(double goal_x, double goal_y);
    double getAngleToGoal(double goal_x, double goal_y);
    bool moveToNextWaypoint();
};

RobotNavigator::RobotNavigator() : current_waypoint_(0), pose_initialized_(false), avoidance_state_(NONE), turn_direction(1) {
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    odom_sub_ = nh_.subscribe("/odom", 1000, &RobotNavigator::odomCallback, this);
    laser_sub_ = nh_.subscribe("/hokuyo", 1000, &RobotNavigator::laserCallback, this);

    std::string world_file;
    nh_.param<std::string>("world_file", world_file, "src/robot_garden/p3at_gazebo/worlds/garden.world");

    std::cout << "World file location " + world_file << std::endl;
    flower_pots = getFlowerPotLocations(world_file);

    ROS_INFO("Loaded %lu waypoints", flower_pots.size());

    for (const auto& waypoint : GARDEN_1_WAYPOINTS) {
        points.push_back({waypoint.x, waypoint.y});
    }
}

bool RobotNavigator::isPathClear() {
    if (laser_ranges_.empty()) return false;

    int center = laser_ranges_.size() / 2;
    int start = center - FRONT_RAYS / 2;
    int end = center + FRONT_RAYS / 2;

    for (int i = start; i < end; i++) {
        if (laser_ranges_[i] < CHECK_AHEAD_DISTANCE && laser_ranges_[i] > 0.0) {
            return false;
        }
    }
    return true;
}


bool RobotNavigator::skipNearbyWaypoint() {
    if (current_waypoint_ < points.size()) {
        if (isNearFlowerPot(points[current_waypoint_].first, points[current_waypoint_].second)) {
            ROS_INFO("Skipping waypoint %lu as it's too close to a flower pot", current_waypoint_);
            current_waypoint_++;
            return true;
        } 
    }
    return false;
}

bool RobotNavigator::isNearFlowerPot(double x, double y) {
    for (const auto& point : flower_pots) {
        double distance = std::sqrt(
            std::pow(x - point.first, 2) +
            std::pow(y - point.second, 2)
        );
        if (distance < FLOWER_POT_NEAR_RADIUS) {
            return true;
        }
    }
    return false;
}

int RobotNavigator::determineOptimalTurnDirection(double obstacle_angle) {
    if (current_waypoint_ >= points.size()) {
        return 1;
    }

    double goal_x = points[current_waypoint_].first;
    double goal_y = points[current_waypoint_].second;

    double angle_to_goal = std::atan2(goal_y - current_y_, goal_x - current_x_) - current_yaw_;
    angle_to_goal = std::atan2(std::sin(angle_to_goal), std::cos(angle_to_goal));

    double turn_left_diff = std::abs(angle_to_goal - (obstacle_angle + M_PI / 2));
    double turn_right_diff = std::abs(angle_to_goal - (obstacle_angle - M_PI / 2));

    turn_left_diff = std::abs(std::atan2(std::sin(turn_left_diff), std::cos(turn_left_diff)));
    turn_right_diff = std::abs(std::atan2(std::sin(turn_right_diff), std::cos(turn_right_diff)));

    return (turn_left_diff < turn_right_diff) ? 1 : -1;
}

void RobotNavigator::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    int center = msg->ranges.size() / 2;
    int start = center - FRONT_RAYS / 2;
    int end = center + FRONT_RAYS / 2;

    bool obstacle_in_front = false;
    double min_distance = std::numeric_limits<double>::max();
    int obstacle_index = -1;

    for (int i = start; i < end; i++) {
        if (msg->ranges[i] < OBSTACLE_DISTANCE && msg->ranges[i] > 0.0) {
            double angle = msg->angle_min + i * msg->angle_increment + current_yaw_;
            double point_x = current_x_ + msg->ranges[i] * cos(angle);
            double point_y = current_y_ + msg->ranges[i] * sin(angle);

            if (!isNearFlowerPot(point_x, point_y)) {
                obstacle_in_front = true;
                if (msg->ranges[i] < min_distance) {
                    min_distance = msg->ranges[i];
                    obstacle_index = i;
                }
            }
        }
    }

    if (avoidance_state_ == NONE && obstacle_in_front) {
        obstacle_angle_ = msg->angle_min + (obstacle_index * msg->angle_increment);

        ROS_INFO("Obstacle angle: %f", obstacle_angle_);
        turn_direction = determineOptimalTurnDirection(obstacle_angle_);

        ROS_INFO("Obstacle angle: %f, Turning direction: %d", obstacle_angle_, turn_direction);

        if (avoidance_state_ == NONE) {
            avoidance_state_ = TURNING;
            avoidance_start_time_ = ros::Time::now();
            ROS_INFO("Starting obstacle avoidance - turning phase");
        }
    }

    obstacle_detected_ = obstacle_in_front;
}

void RobotNavigator::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;

    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, current_yaw_);

    pose_initialized_ = true;
}

double RobotNavigator::getDistanceToGoal(double goal_x, double goal_y) {
    double distance = std::sqrt(std::pow(goal_x - current_x_, 2) +
        std::pow(goal_y - current_y_, 2));
    ROS_INFO("Distance to goal: %f", distance);
    return distance;
}

double RobotNavigator::getAngleToGoal(double goal_x, double goal_y) {
    double angle = std::atan2(goal_y - current_y_, goal_x - current_x_) - current_yaw_;
    return std::atan2(std::sin(angle), std::cos(angle));
}

bool RobotNavigator::moveToNextWaypoint() {
    if (current_waypoint_ >= points.size()) {
        return true;
    }

    geometry_msgs::Twist cmd_vel;

    if (avoidance_state_ != NONE) {
        double elapsed_time = (ros::Time::now() - avoidance_start_time_).toSec();

        if (avoidance_state_ == TURNING) {
            if (elapsed_time < TURN_DURATION) {
                double angle_factor = std::abs(std::cos(obstacle_angle_));
                double turn_strength = MIN_TURN_SPEED +
                    (MAX_TURN_SPEED - MIN_TURN_SPEED) * angle_factor;

                cmd_vel.angular.z = turn_strength * turn_direction;

                ROS_INFO("Turning with strength: %f, angle factor: %f",
                    turn_strength, angle_factor);
            }
            else {
                avoidance_state_ = MOVING_FORWARD;
                avoidance_start_time_ = ros::Time::now();
                ROS_INFO("Path clear, moving forward");
            }
        }
        else if (avoidance_state_ == MOVING_FORWARD) {
            if (elapsed_time < FORWARD_DURATION) {
                cmd_vel.linear.x = LINEAR_SPEED;
            }
            else {
                avoidance_state_ = NONE;
                ROS_INFO("Obstacle avoidance complete");
            }
        }

        cmd_vel_pub_.publish(cmd_vel);
        return false;
    }

    ROS_INFO("Following waypoint");

    double goal_x = points[current_waypoint_].first;
    double goal_y = points[current_waypoint_].second;

    double distance = getDistanceToGoal(goal_x, goal_y);
    double angle = getAngleToGoal(goal_x, goal_y);

    if (std::abs(angle) > ANGULAR_THRESHOLD) {
        cmd_vel.angular.z = angle > 0 ? ANGULAR_SPEED : -ANGULAR_SPEED;
    }
    else if (distance > DISTANCE_THRESHOLD) {
        cmd_vel.linear.x = LINEAR_SPEED;
        cmd_vel.angular.z = 0.2 * angle;
    }
    else {
        ROS_INFO("Reached waypoint %lu", current_waypoint_);
        current_waypoint_++;
        return false;
    }

    cmd_vel_pub_.publish(cmd_vel);
    return false;
}

void RobotNavigator::navigate() {
    ros::Rate rate(10);

    while (ros::ok() && !pose_initialized_) {
        ros::spinOnce();
        rate.sleep();
    }

    while (ros::ok()) {
        if (moveToNextWaypoint()) {
            ROS_INFO("All waypoints reached!");
            break;
        }

        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "waypoint_following_custom");
    RobotNavigator navigator;
    navigator.navigate();
    return 0;
}