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

#include <ros/package.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/Twist.h>
#include <vector>
#include <tf/transform_listener.h>

class RobotController {
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

    // save the pose pairs
    std::vector<std::pair<geometry_msgs::Pose, geometry_msgs::Pose>> pose_pairs;

    // paths to follow
    std::vector<nav_msgs::Path> paths;

    // Get the robot's current position as the first start
    geometry_msgs::Pose current_pose;

    
    AvoidanceState avoidance_state_;
    ros::Time avoidance_start_time_;
    const double TURN_DURATION = 4.0;   
    const double FORWARD_DURATION = 6.0;

    std::vector<std::pair<float, float>> waypoints_;
    size_t current_waypoint_;
    
    double current_x_;
    double current_y_;
    //double current_angular_z_;
    double current_yaw_;
    bool pose_initialized_;
    bool obstacle_detected_;
    bool is_path_clear_;

    int turn_direction = 1;
    
    std::vector<float> laser_ranges_;

    std::vector<std::pair<float, float>> points = {
            {-2.5f, -2.0f},
            {-1.5f, -3.0f},
            {-3.0f, -5.0f},
            {-8.0f, -5.0f},
            {-7.0f, -3.5f},
            {-7.0f, 0.0f},
            {-7.5f, 2.0f}
    };
    
    // Control parameters
    const double DISTANCE_THRESHOLD = 0.5;
    const double ANGULAR_THRESHOLD = 0.5;
    const double LINEAR_SPEED = 0.3;
    const double ANGULAR_SPEED = 0.6;
    const double WAYPOINT_IGNORE_RADIUS = 2.0;

    const double OBSTACLE_DISTANCE = 0.5;    // Detect obstacles 0.5 meter ahead
    const double CHECK_AHEAD_DISTANCE = 0.2;  // Look further ahead
    const int FRONT_RAYS = 10;               // Number of rays to check in front
    const double MAX_TURN_SPEED = 0.5;
    const double MIN_TURN_SPEED = 0.3;  
    double obstacle_angle_;           

    // Add new members for path following
    size_t current_path_index_;
    size_t current_pose_index_;
    
    // Modified parameters for path following
    const double PATH_POSE_THRESHOLD = 0.3;  // Distance threshold for reaching path poses
    const double PATH_ANGULAR_THRESHOLD = 0.4;  // Angular threshold for path poses
  

public:
    RobotController() : current_waypoint_(0), current_path_index_(0), current_pose_index_(0), pose_initialized_(false), avoidance_state_(NONE) {
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
        odom_sub_ = nh_.subscribe("/odom", 1000, &RobotController::odomCallback, this);
        laser_sub_ = nh_.subscribe("/hokuyo", 1000, &RobotController::laserCallback, this);
        
        std::string world_file;
        nh_.param<std::string>("world_file", world_file, "src/robot_garden/p3at_gazebo/worlds/garden.world");

        std::cout << "World file location " + world_file << std::endl;
        waypoints_ = getFlowerPotLocations(world_file);
        
        //std::reverse(waypoints_.begin(), waypoints_.end());
        
        ROS_INFO("Loaded %lu flower locations", waypoints_.size());

        tf::TransformListener listener;
        tf::StampedTransform transform;

        try
        {
            listener.waitForTransform("map", "base_footprint", ros::Time(0), ros::Duration(10.0));
            listener.lookupTransform("map", "base_footprint", ros::Time(0), transform);

            current_pose.position.x = transform.getOrigin().x();
            current_pose.position.y = transform.getOrigin().y();
            current_pose.position.z = transform.getOrigin().z();
            current_pose.orientation.x = transform.getRotation().x();
            current_pose.orientation.y = transform.getRotation().y();
            current_pose.orientation.z = transform.getRotation().z();
            current_pose.orientation.w = transform.getRotation().w();
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }

        // Use current_pose as the start for the first goal
        geometry_msgs::Pose start_pose = current_pose;

        for (const auto& location : points)
        {
            geometry_msgs::Pose goal_pose;
            goal_pose.position.x = location.first;
            goal_pose.position.y = location.second;
            goal_pose.position.z = 0.0; // Assuming flat ground, adjust if necessary

            // Add the start and goal pair to the list
            pose_pairs.push_back({start_pose, goal_pose});

            // Update start_pose for the next goal
            start_pose = goal_pose;
        }

        // First, compute all paths and smooth them
        for (const auto& pose_pair : pose_pairs)
        {
            const auto& start = pose_pair.first;
            const auto& goal = pose_pair.second;

            // Plan the path with Move Base
            nav_msgs::Path global_path = planPathWithMoveBase(start, goal);

            if (!global_path.poses.empty()) {  // Smooth the path if it's valid
                // Smooth the path using a custom smoothing function
                // nav_msgs::Path smoothed_path = smoothPath(global_path);
                paths.push_back(global_path);
            }
            else {
                ROS_ERROR("Failed to plan a path from (%f, %f) to (%f, %f)", start.position.x, start.position.y, goal.position.x, goal.position.y);
            }
        }

        ROS_INFO("Planned %lu paths with total waypoints:", paths.size());
        for (size_t i = 0; i < paths.size(); i++) {
            ROS_INFO("Path %lu has %lu poses", i, paths[i].poses.size());
        }
    }

    std::vector<std::pair<float, float>> getFlowerPotLocations(const std::string& filePath) {
        std::vector<std::pair<float, float>> flowerPotLocations;
        
        tinyxml2::XMLDocument doc;
        if (doc.LoadFile(filePath.c_str()) != tinyxml2::XMLError::XML_SUCCESS) {
            std::cerr << "Error parsing the XML file." << std::endl;
            return flowerPotLocations;
        }

        tinyxml2::XMLElement* sdf = doc.RootElement();
        if (!sdf) {
            std::cerr << "Cannot find sdf element." << std::endl;
            return flowerPotLocations;
        }

        tinyxml2::XMLElement* world = sdf->FirstChildElement("world");
        if (!world) {
            std::cerr << "Cannot find world element." << std::endl;
            return flowerPotLocations;
        }

        tinyxml2::XMLElement* state = world->FirstChildElement("state");
        if (!state) {
            std::cerr << "Cannot find state element." << std::endl;
            return flowerPotLocations;
        }


        // Get all "model" elements inside it
        tinyxml2::XMLElement* model = state->FirstChildElement("model");
        while (model) {
            const char* name = model->Attribute("name");
            if (name && std::string(name).find("Cole_Hardware") != std::string::npos) {
                tinyxml2::XMLElement* pose = model->FirstChildElement("pose");
                if (pose) {
                    std::string poseStr = pose->GetText();

                    //flowerPotLocations.emplace_back(poseStr.at(0), poseStr.at(1));
                    
                    std::istringstream iss(poseStr);
                    float x, y, z, roll, pitch, yaw;
                    if (iss >> x >> y >> z >> roll >> pitch >> yaw) {
                        flowerPotLocations.emplace_back(x, y);
                    }
                }
            }
            model = model->NextSiblingElement("model");
        }

        if (flowerPotLocations.empty()) {
            std::cout << "No flower pots found in the file." << std::endl;
        }

        return flowerPotLocations;
    }

    // Function to plan a path using Move Base's make_plan service
    nav_msgs::Path planPathWithMoveBase(const geometry_msgs::Pose& start, const geometry_msgs::Pose& goal)
    {
        ros::NodeHandle nh;
        // Use simulated time
        nh.setParam("use_sim_time", true);
        ros::ServiceClient make_plan_client = nh.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");
        make_plan_client.waitForExistence();

        nav_msgs::GetPlan srv;
        srv.request.start.header.frame_id = "map";
        srv.request.start.header.stamp = ros::Time::now();
        srv.request.start.pose = start;

        srv.request.goal.header.frame_id = "map";
        srv.request.goal.header.stamp = ros::Time::now();
        srv.request.goal.pose = goal;

        srv.request.tolerance = 0.0; // Set tolerance as needed

        nav_msgs::Path planned_path;
        if (make_plan_client.call(srv)) {
            ROS_INFO("Received plan with %lu waypoints", srv.response.plan.poses.size());
            // You can process the plan here, e.g., print the waypoints
            for (size_t i = 0; i < srv.response.plan.poses.size(); ++i) {
                ROS_INFO("Waypoint %lu: x=%.2f, y=%.2f", i, 
                        srv.response.plan.poses[i].pose.position.x,
                        srv.response.plan.poses[i].pose.position.y);
            }
            planned_path = srv.response.plan;
        } else {
            ROS_ERROR("Failed to call service make_plan");
        }

        return planned_path;
    }

    void sendGoal(const geometry_msgs::Point& position, const geometry_msgs::Quaternion& orientation) {
        move_base_msgs::MoveBaseGoal goal;
        
        // Set up the frame parameters
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        
        // Set the position
        goal.target_pose.pose.position = position;
        goal.target_pose.pose.orientation = orientation;
        
        ROS_INFO("Sending goal: x=%f, y=%f", position.x, position.y);
        ac_->sendGoal(goal);
    }

    void skipNearbyWaypoint() {
        if (current_waypoint_ < waypoints_.size()) {
            if (isNearWaypoint(waypoints_[current_waypoint_].first, 
                             waypoints_[current_waypoint_].second)) {
                ROS_INFO("Skipping waypoint %lu as it's too close", current_waypoint_);
                current_waypoint_++;
            } 
        }
    }

    bool isNearWaypoint(double x, double y) {
        for (const auto& point : waypoints_) {
            double distance = std::sqrt(
                std::pow(x - point.first, 2) + 
                std::pow(y - point.second, 2)
            );
            if (distance < WAYPOINT_IGNORE_RADIUS) {
                return true;
            }
        }
        return false;
    }

    int determineOptimalTurnDirection(double obstacle_angle) {
        if (current_waypoint_ >= points.size()) {
            return 1;
        }

        double goal_x = points[current_waypoint_].first;
        double goal_y = points[current_waypoint_].second;

        double angle_to_goal = std::atan2(goal_y - current_y_, goal_x - current_x_) - current_yaw_;
        
        angle_to_goal = std::atan2(std::sin(angle_to_goal), std::cos(angle_to_goal));

        double turn_left_diff = std::abs(angle_to_goal - (obstacle_angle + M_PI/2));
        double turn_right_diff = std::abs(angle_to_goal - (obstacle_angle - M_PI/2));

        turn_left_diff = std::abs(std::atan2(std::sin(turn_left_diff), std::cos(turn_left_diff)));
        turn_right_diff = std::abs(std::atan2(std::sin(turn_right_diff), std::cos(turn_right_diff)));

        return (turn_left_diff < turn_right_diff) ? 1 : -1;
    }

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        int center = msg->ranges.size() / 2;
        int start = center - FRONT_RAYS/2;
        int end = center + FRONT_RAYS/2;
        
        bool obstacle_in_front = false;
        double min_distance = std::numeric_limits<double>::max();
        int obstacle_index = -1;

        for (int i = start; i < end; i++) {
            // if (msg->ranges[i] < OBSTACLE_DISTANCE && msg->ranges[i] > 0.0) {
            //     obstacle_in_front = true;
            //     if (msg->ranges[i] < min_distance) {
            //         min_distance = msg->ranges[i];
            //         obstacle_index = i;
            //     }
            // }

            if (msg->ranges[i] < OBSTACLE_DISTANCE && msg->ranges[i] > 0.0) {
                // Convert laser reading to world coordinates
                double angle = msg->angle_min + i * msg->angle_increment + current_yaw_;
                double point_x = current_x_ + msg->ranges[i] * cos(angle);
                double point_y = current_y_ + msg->ranges[i] * sin(angle);

                // Only consider as obstacle if not near waypoint
                if (!isNearWaypoint(point_x, point_y)) {
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
            //turn_direction = (obstacle_index < center) ? -1 : 1;
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

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
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

    double getDistanceToGoal(double goal_x, double goal_y) {
        double distance = std::sqrt(std::pow(goal_x - current_x_, 2) + 
                        std::pow(goal_y - current_y_, 2));
        ROS_INFO("Distance to goal: %f", distance);
        return distance;
    }

    double getAngleToGoal(double goal_x, double goal_y) {
        double angle = std::atan2(goal_y - current_y_, goal_x - current_x_) - current_yaw_;
        return std::atan2(std::sin(angle), std::cos(angle));
    }

    bool moveToNextWaypoint() {
        ROS_INFO("In Moving to Waypoint first time, paths size %lu", paths.size());
        if (current_path_index_ >= paths.size()) {
            ROS_INFO("All paths completed");
            return true;
        }

        // Get current path and pose
        const nav_msgs::Path& current_path = paths[current_path_index_];
        
        if (current_pose_index_ >= current_path.poses.size()) {
            // Move to next path
            current_path_index_++;
            current_pose_index_ = 0;
            ROS_INFO("Moving to next path: %lu", current_path_index_);
            return moveToNextWaypoint();  // Recursive call to handle next path
        }

        geometry_msgs::Twist cmd_vel;
        
        // Handle obstacle avoidance sequence
        if (avoidance_state_ != NONE) {
            double elapsed_time = (ros::Time::now() - avoidance_start_time_).toSec();
            if (avoidance_state_ == TURNING) {
                if (elapsed_time < TURN_DURATION) {
                    double angle_factor = std::abs(std::cos(obstacle_angle_));
                    // More head-on collisions (angle_factor close to 1) result in stronger turns
                    double turn_strength = MIN_TURN_SPEED + 
                        (MAX_TURN_SPEED - MIN_TURN_SPEED) * angle_factor;
                    
                    cmd_vel.angular.z = turn_strength * turn_direction;
                    
                    ROS_INFO("Turning with strength: %f, angle factor: %f", 
                            turn_strength, angle_factor);
                } else {
                    avoidance_state_ = MOVING_FORWARD;
                    avoidance_start_time_ = ros::Time::now();
                    ROS_INFO("Path clear, moving forward");
                }
            }
            else if (avoidance_state_ == MOVING_FORWARD) {
                if (elapsed_time < FORWARD_DURATION) {
                    cmd_vel.linear.x = LINEAR_SPEED;
                } else {
                    avoidance_state_ = NONE;
                    ROS_INFO("Obstacle avoidance complete");
                    // Don't return here - continue with normal path following
                }
            }
            cmd_vel_pub_.publish(cmd_vel);
            if (avoidance_state_ != NONE) {
                return false;  // Only return if still avoiding
            }
        }

        const geometry_msgs::PoseStamped& target_pose = current_path.poses[current_pose_index_];
        
        double goal_x = target_pose.pose.position.x;
        double goal_y = target_pose.pose.position.y;
        
        // Calculate target yaw from quaternion
        tf::Quaternion q(
            target_pose.pose.orientation.x,
            target_pose.pose.orientation.y,
            target_pose.pose.orientation.z,
            target_pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, target_yaw;
        m.getRPY(roll, pitch, target_yaw);
        
        ROS_INFO("Following path %lu, pose %lu", 
                current_path_index_, current_pose_index_);  // Add current position logging
        
        double distance = getDistanceToGoal(goal_x, goal_y);
        double angle = getAngleToGoal(goal_x, goal_y);
        
        ROS_INFO("Distance to goal: %f, Angle to goal: %f", distance, angle);  // Add debug info
        
        // Follow the path pose
        if (std::abs(angle) > PATH_ANGULAR_THRESHOLD) {
            // Turn towards the target pose
            cmd_vel.angular.z = angle > 0 ? ANGULAR_SPEED : -ANGULAR_SPEED;
            ROS_INFO("Turning with angular velocity: %f", cmd_vel.angular.z);
        }
        else if (distance > PATH_POSE_THRESHOLD) {
            // Move towards the target pose
            cmd_vel.linear.x = LINEAR_SPEED;
            cmd_vel.angular.z = 0.2 * angle;  // Proportional control for orientation
            ROS_INFO("Moving with linear velocity: %f, angular correction: %f", 
                    cmd_vel.linear.x, cmd_vel.angular.z);
        }
        else {
            // Reached current pose in the path
            ROS_INFO("Reached pose %lu in path %lu", current_pose_index_, current_path_index_);
            current_pose_index_++;
            cmd_vel.linear.x = 0.0;  // Stop the robot
            cmd_vel.angular.z = 0.0;
            cmd_vel_pub_.publish(cmd_vel);
            return false;
        }
        
        cmd_vel_pub_.publish(cmd_vel);
        return false;
    }

    void run() {
        ros::Rate rate(20);

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
        
        // geometry_msgs::Twist cmd_vel;
        // cmd_vel_pub_.publish(cmd_vel);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "p3at");
    RobotController controller;
    controller.run();
    return 0;
}