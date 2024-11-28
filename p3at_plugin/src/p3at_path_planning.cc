#include <ros/ros.h>
#include <ros/package.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/Twist.h>
#include <vector>
#include "get_flower_locations.h"
#include <tf/transform_listener.h>
#include <ros/package.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

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

// Simple path smoothing function (e.g., averaging nearby waypoints)
nav_msgs::Path smoothPath(const nav_msgs::Path& path)
{
    nav_msgs::Path smoothed_path;
    smoothed_path.header = path.header;

    if (path.poses.size() < 3) {
        ROS_WARN("Path has fewer than 3 waypoints. Skipping smoothing.");
        return path;  // Return the path as is if there are not enough points to smooth
    }
    
    // Example smoothing: a very simple smoothing approach by averaging adjacent points
    for (size_t i = 1; i < path.poses.size() - 1; ++i)
    {
        geometry_msgs::PoseStamped smoothed_pose;
        smoothed_pose.pose.position.x = (path.poses[i-1].pose.position.x + path.poses[i].pose.position.x + path.poses[i+1].pose.position.x) / 3.0;
        smoothed_pose.pose.position.y = (path.poses[i-1].pose.position.y + path.poses[i].pose.position.y + path.poses[i+1].pose.position.y) / 3.0;
        smoothed_pose.pose.position.z = (path.poses[i-1].pose.position.z + path.poses[i].pose.position.z + path.poses[i+1].pose.position.z) / 3.0;
        
        smoothed_path.poses.push_back(smoothed_pose);
    }

    return smoothed_path;
}

// Function to request a path to a goal
void planPathToGoal(const geometry_msgs::Pose& goal)
{
    // Create the MoveBase action client
    MoveBaseClient move_base_client("move_base", true);

    // Wait for the action server to be available
    move_base_client.waitForServer();

    move_base_msgs::MoveBaseGoal move_goal;
    move_goal.target_pose.header.frame_id = "map";
    move_goal.target_pose.header.stamp = ros::Time::now();
    move_goal.target_pose.pose = goal;

    // Send the goal to the move_base action server
    move_base_client.sendGoal(move_goal);

    // Wait for the result from MoveBase
    move_base_client.waitForResult(ros::Duration(60.0));

    // // Get the path from the result
    // nav_msgs::Path path;
    if (move_base_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Goal reached!");

        // If the goal is reached, return the path (MoveBase handles this)
        // path = move_base_client.getResult()->plan;
    }
    else
    {
        ROS_ERROR("Failed to reach the goal.");
        // return path;
    }
}

// Function to execute trajectory by publishing velocity commands
void executeTrajectory(const nav_msgs::Path& path, ros::Publisher& cmd_vel_pub)
{
    if (path.poses.empty()) {
        ROS_ERROR("Path is empty, cannot execute trajectory.");
        return;
    }

    ros::Rate rate(10);
    for (const auto& pose_stamped : path.poses) {
        geometry_msgs::Pose pose = pose_stamped.pose;
        ROS_INFO("Executing waypoint: x=%.2f, y=%.2f", pose.position.x, pose.position.y);
        // Call planToGoal to get the path to the next waypoint
        // nav_msgs::Path path_to_goal = planPathToGoal(pose);
        planPathToGoal(pose);
        rate.sleep();
    }

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_base_amcl_integration");
    ros::NodeHandle nh;
    // Use simulated time
    nh.setParam("use_sim_time", true);

    // Print simulated time
    ROS_INFO("Current simulated time: %f", ros::Time::now().toSec());

    // Publisher for sending velocity commands
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // Get the robot's current position as the first start
    geometry_msgs::Pose current_pose;

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

    // Get the path of a package
    std::string package_path = ros::package::getPath("p3at_gazebo");

    if (package_path.empty()) {
        ROS_ERROR("Could not find package 'p3at_gazebo'.");
        return 1; // Exit if package path is not found
    }

    // Load the world file
    std::string world_file = package_path + "/worlds/garden.world";
    std::cout << "World file location " + world_file << std::endl;
    
     // Load waypoints from file
    
    std::vector<std::pair<float, float>> waypoints;
    waypoints = getFlowerPotLocations(world_file);

    if (waypoints.empty()) {
        ROS_ERROR("No waypoints loaded from the world file.");
        return 1; // Exit if no waypoints are loaded
    }

    ROS_INFO("Loaded %lu waypoints", waypoints.size());

    std::vector<std::pair<geometry_msgs::Pose, geometry_msgs::Pose>> pose_pairs;

    // Use current_pose as the start for the first goal
    geometry_msgs::Pose start_pose = current_pose;

    for (const auto& location : waypoints)
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

    std::vector<nav_msgs::Path> paths;

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

    // Then, execute all the smoothed paths
    for (const auto& path : paths)
    {
        executeTrajectory(path, cmd_vel_pub);
    }

    ros::spin();
    return 0;
}