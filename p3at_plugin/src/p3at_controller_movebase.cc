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
#include <geometry_msgs/Pose.h>
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

    // Define a client for to send goal requests to the move base server through a SimpleActionClient
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

    // save the pose pairs
    std::vector<std::pair<geometry_msgs::Pose, geometry_msgs::Pose>> pose_pairs;

    // paths to follow
    std::vector<nav_msgs::Path> paths;

    // Get the robot's current position as the first start
    geometry_msgs::Pose current_pose;

    // Action server
    std::unique_ptr<MoveBaseClient> ac_;

    // Add new private member for number of samples
    const size_t NUM_SAMPLES = 5;
    
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
        // laser_sub_ = nh_.subscribe("/hokuyo", 1000, &RobotController::laserCallback, this);

        // Initialize the MoveBaseClient
        ac_ = std::make_unique<MoveBaseClient>("move_base", true);

        // Wait for the action server to come up
        while(!ac_->waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
        }
        
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

        ROS_INFO("Current pose: %f and %f", current_pose.position.x, current_pose.position.y);

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

        std::vector<nav_msgs::Path> sampled_paths;
        for (const auto& path : paths) {
            if (!path.poses.empty()) {
                nav_msgs::Path sampled_path = samplePath(path);
                sampled_paths.push_back(sampled_path);
                ROS_INFO("Sampled path from %lu poses to %lu poses", 
                    path.poses.size(), sampled_path.poses.size());
            }
        }
        
        // Replace original paths with sampled ones
        paths = std::move(sampled_paths);

        ROS_INFO("Created %lu sampled paths", paths.size());
        for (size_t i = 0; i < paths.size(); i++) {
            ROS_INFO("Sampled path %lu has %lu poses", i, paths[i].poses.size());
        }

    }

    // // Add new helper function to sample poses from a path
    // nav_msgs::Path samplePath(const nav_msgs::Path& original_path) {
    //     nav_msgs::Path sampled_path;
    //     sampled_path.header = original_path.header;
        
    //     if (original_path.poses.size() <= NUM_SAMPLES) {
    //         return original_path;  // Return original if it has fewer poses than needed
    //     }

    //     // Calculate the stride for even sampling
    //     double stride = static_cast<double>(original_path.poses.size() - 1) / (NUM_SAMPLES - 1);
        
    //     for (size_t i = 0; i < NUM_SAMPLES; i++) {
    //         size_t index = static_cast<size_t>(i * stride);
    //         if (i == NUM_SAMPLES - 1) {
    //             index = original_path.poses.size() - 1;  // Ensure we get the last pose
    //         }
    //         sampled_path.poses.push_back(original_path.poses[index]);
    //     }

    //     return sampled_path;
    // }

    nav_msgs::Path samplePath(const nav_msgs::Path& original_path) {
        
        nav_msgs::Path sampled_path;
        sampled_path.header = original_path.header;
        
        if (original_path.poses.size() <= NUM_SAMPLES) {
            return original_path;
        }

        // Calculate the stride for even sampling
        double stride = static_cast<double>(original_path.poses.size() - 1) / (NUM_SAMPLES - 1);
        
        for (size_t i = 0; i < NUM_SAMPLES; i++) {
            size_t current_index = static_cast<size_t>(i * stride);
            if (i == NUM_SAMPLES - 1) {
                current_index = original_path.poses.size() - 1;
            }

            geometry_msgs::PoseStamped pose = original_path.poses[current_index];
            
            // Calculate the orientation based on the next point in the path
            if (i < NUM_SAMPLES - 1) {
                size_t next_index = static_cast<size_t>((i + 1) * stride);
                if (next_index >= original_path.poses.size()) {
                    next_index = original_path.poses.size() - 1;
                }
                
                pose.pose.orientation = calculateOrientation(
                    pose.pose.position,
                    original_path.poses[next_index].pose.position
                );
            } else {
                // For the last point, keep the final goal orientation
                pose.pose.orientation = original_path.poses.back().pose.orientation;
            }
            
            sampled_path.poses.push_back(pose);
        }

        return sampled_path;
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

    geometry_msgs::Quaternion calculateOrientation(const geometry_msgs::Point& current,
                                                 const geometry_msgs::Point& next) {
        double dx = next.x - current.x;
        double dy = next.y - current.y;
        double yaw = std::atan2(dy, dx);
        
        geometry_msgs::Quaternion orientation;
        orientation.x = 0.0;
        orientation.y = 0.0;
        orientation.z = std::sin(yaw / 2.0);
        orientation.w = std::cos(yaw / 2.0);
        return orientation;
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

    // void sendGoal(const geometry_msgs::Point& position, const geometry_msgs::Quaternion& orientation) {
    //     move_base_msgs::MoveBaseGoal goal;
        
    //     // Set up the frame parameters
    //     goal.target_pose.header.frame_id = "map";
    //     goal.target_pose.header.stamp = ros::Time::now();
        
    //     // Set the position
    //     goal.target_pose.pose.position = position;
    //     goal.target_pose.pose.orientation = orientation;
        
    //     ROS_INFO("Sending goal: x=%f, y=%f", position.x, position.y);
    //     ac_->sendGoal(goal);

    //     // Wait an infinite time for the results
    //     ac->waitForResult();

    //     if (ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    //         ROS_INFO("Hooray, the robot reached goal %d!", i + 1);
    //     } else {
    //         ROS_INFO("The robot failed to reach goal %d", i + 1);
    //     }
    // }


    bool goToGoals() {
        
        if (paths.empty()) {
            ROS_WARN("No paths available for navigation");
            return true;  // Return true since there's nothing to do
        }

        if (current_path_index_ >= paths.size()) {
            ROS_INFO("Completed all paths!");
            return true;
        }

        const nav_msgs::Path& current_path = paths[current_path_index_];

        // if (current_pose_index_ >= current_path.poses.size()) {
        //     current_path_index_++;
        //     current_pose_index_ = 0;
        //     ROS_INFO("Moving to next path %zu", current_path_index_);
        //     return false;  // More paths to process
        // }

        // Get the current goal pose
        //const geometry_msgs::PoseStamped& current_goal = current_path.poses[current_pose_index_];

        executeTrajectory(current_path, cmd_vel_pub_);
        current_path_index_++;
        // // Send the goal and wait for result
        // move_base_msgs::MoveBaseGoal goal;
        // goal.target_pose.header.frame_id = "map";
        // goal.target_pose.pose = current_path.poses[current_pose_index_].pose;
        // goal.target_pose.header.stamp = ros::Time::now();
        
        // ac_->sendGoal(goal);
        
        // // Wait for the result with a timeout
        // ac_->waitForResult();
        
        // actionlib::SimpleClientGoalState state = ac_->getState();
        // if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
        //     ROS_INFO("Reached pose %zu in path %zu", current_pose_index_, current_path_index_);
        //     current_pose_index_++;
        // } else {
        //     ROS_WARN("Failed to reach pose %zu in path %zu: %s", 
        //     current_pose_index_, current_path_index_, state.toString().c_str());
        // }
        

        return false;  // Still have more goals to process
    }

    // void planPathToGoal(const geometry_msgs::Pose& goal){
    //     // Create the MoveBase action client
    //     MoveBaseClient move_base_client("move_base", true);

    //     // Wait for the action server to be available
    //     move_base_client.waitForServer();

    //     move_base_msgs::MoveBaseGoal move_goal;
    //     move_goal.target_pose.header.frame_id = "map";
    //     move_goal.target_pose.header.stamp = ros::Time::now();
    //     move_goal.target_pose.pose = goal;

    //     // Send the goal to the move_base action server
    //     move_base_client.sendGoal(move_goal);

    //     // Wait for the result from MoveBase
    //     move_base_client.waitForResult(ros::Duration(60.0));

    //     // // Get the path from the result
    //     // nav_msgs::Path path;
    //     if (move_base_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    //     {
    //         //ROS_INFO("Goal reached!");

    //         // If the goal is reached, return the path (MoveBase handles this)
    //         // path = move_base_client.getResult()->plan;
    //     }
    //     else
    //     {
    //         ROS_ERROR("Failed to reach the goal.");
    //         // return path;
    //     }
    // }

    void planPathToGoal(const geometry_msgs::Pose& goal) {
        
        MoveBaseClient move_base_client("move_base", true);
        move_base_client.waitForServer();

        move_base_msgs::MoveBaseGoal move_goal;
        move_goal.target_pose.header.frame_id = "map";
        move_goal.target_pose.header.stamp = ros::Time::now();
        move_goal.target_pose.pose = goal;

        // Add tolerance for orientation
        move_base_client.sendGoal(move_goal, 
            actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>::SimpleDoneCallback(),
            actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>::SimpleActiveCallback(),
            actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>::SimpleFeedbackCallback());

        // Reduced wait time and added orientation tolerance parameter
        move_base_client.waitForResult(ros::Duration(30.0));

        if (move_base_client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_WARN("Failed to reach the goal, moving to next waypoint");
        }
    }

    void executeTrajectory(const nav_msgs::Path& path, ros::Publisher& cmd_vel_pub){
        
        if (path.poses.empty()) {
            ROS_ERROR("Path is empty, cannot execute trajectory.");
            return;
        }

        ros::Rate rate(10);
        for (const auto& pose_stamped : path.poses) {
            geometry_msgs::Pose pose = pose_stamped.pose;
            //ROS_INFO("Executing waypoint: x=%.2f, y=%.2f", pose.position.x, pose.position.y);
            // Call planToGoal to get the path to the next waypoint
            // nav_msgs::Path path_to_goal = planPathToGoal(pose);
            planPathToGoal(pose);
            rate.sleep();
        }

    }

    void run() {
        ros::Rate rate(20);

        while (ros::ok() && !pose_initialized_) {
            ros::spinOnce();
            rate.sleep();
        }

        bool all_goals_reached = false;
        while (ros::ok() && !all_goals_reached) {
            all_goals_reached = goToGoals();
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