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

class RobotController {
private:
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber laser_sub_;
    
    std::vector<std::pair<float, float>> waypoints_;
    size_t current_waypoint_;
    
    double current_x_;
    double current_y_;
    //double current_angular_z_;
    double current_yaw_;
    bool pose_initialized_;
    
    std::vector<float> laser_ranges_;
    
    // Control parameters
    const double DISTANCE_THRESHOLD = 0.2;
    const double ANGULAR_THRESHOLD = 0.1;
    const double LINEAR_SPEED = 0.3;
    const double ANGULAR_SPEED = 0.5;

public:
    RobotController() : current_waypoint_(0), pose_initialized_(false) {
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
        odom_sub_ = nh_.subscribe("/odom", 1000, &RobotController::odomCallback, this);
        laser_sub_ = nh_.subscribe("/hokuyo", 1000, &RobotController::laserCallback, this);
        
        std::string world_file;
        nh_.param<std::string>("world_file", world_file, "src/robot_garden/p3at_gazebo/worlds/garden.world");

        std::cout << "World file location " + world_file << std::endl;
        waypoints_ = getFlowerPotLocations(world_file);
        
        ROS_INFO("Loaded %lu waypoints", waypoints_.size());
    }

    std::vector<std::pair<float, float>> getFlowerPotLocations(const std::string& filePath) {
        std::vector<std::pair<float, float>> flowerPotLocations;
        
        // Get the root document
        tinyxml2::XMLDocument doc;
        if (doc.LoadFile(filePath.c_str()) != tinyxml2::XMLError::XML_SUCCESS) {
            std::cerr << "Error parsing the XML file." << std::endl;
            return flowerPotLocations;
        }

        // Start from the root
        tinyxml2::XMLElement* sdf = doc.RootElement();
        if (!sdf) {
            std::cerr << "Cannot find sdf element." << std::endl;
            return flowerPotLocations;
        }

        // Get the first child "world"
        tinyxml2::XMLElement* world = sdf->FirstChildElement("world");
        if (!world) {
            std::cerr << "Cannot find world element." << std::endl;
            return flowerPotLocations;
        }

        // Get the second child "state"
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

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        // Store laser readings
        laser_ranges_ = msg->ranges;
        
        // Debug print - first and last readings
        if (!laser_ranges_.empty()) {
            ROS_DEBUG("First laser reading: %.2f, Last laser reading: %.2f",
                     laser_ranges_.front(), laser_ranges_.back());
        }
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
        return std::sqrt(std::pow(goal_x - current_x_, 2) + 
                        std::pow(goal_y - current_y_, 2));
    }

    double getAngleToGoal(double goal_x, double goal_y) {
        double angle = std::atan2(goal_y - current_y_, goal_x - current_x_) - current_yaw_;
        return std::atan2(std::sin(angle), std::cos(angle));
    }

    bool moveToNextWaypoint() {
        if (current_waypoint_ >= waypoints_.size()) {
            return true;
        }

        double goal_x = waypoints_[current_waypoint_].first;
        double goal_y = waypoints_[current_waypoint_].second;
        
        double distance = getDistanceToGoal(goal_x, goal_y);
        double angle = getAngleToGoal(goal_x, goal_y);

        geometry_msgs::Twist cmd_vel;

        if (std::abs(angle) > ANGULAR_THRESHOLD) {
            cmd_vel.angular.z = angle > 0 ? ANGULAR_SPEED : -ANGULAR_SPEED;
        }
        else if (distance > DISTANCE_THRESHOLD) {
            cmd_vel.linear.x = LINEAR_SPEED;
            cmd_vel.angular.z = 0.2 * angle;
        }
        else {
            current_waypoint_++;
            ROS_INFO("Reached waypoint %lu", current_waypoint_);
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
        
        geometry_msgs::Twist cmd_vel;
        cmd_vel_pub_.publish(cmd_vel);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "p3at");
    RobotController controller;
    controller.run();
    return 0;
}