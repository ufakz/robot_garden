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
#include <tf/transform_datatypes.h>
#include <ros/package.h>
#include <cmath> 
#include <fstream>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// Function to convert a yaw angle to a quaternion
geometry_msgs::Quaternion yawToQuaternion(double yaw) {
    
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(yaw / 2.0);
    quaternion.w = cos(yaw / 2.0);

    return quaternion;
}

// Define the global array of pose data (start and goal positions and orientations)
struct PoseData {
    double x, y, z;
};

PoseData pose_data[] = {
    {-2.5, -2.0, 0.0},
    {-1.5, -3.0, 0.0},
    {-3.0, -5.0, 0.0},
    {-8.0, -5.0, 0.0},
    {-7.0, -3.5, 0.0},
    {-7.0, 0.0, 0.0},
    {-7.5, 2.0, 0.0}

};

// Function to create a pose from x, y, z
geometry_msgs::Pose createPose(double x, double y, double z) {
    geometry_msgs::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    return pose;
}

geometry_msgs::Quaternion calculateOrientation(const geometry_msgs::Point& current,
                                                 const geometry_msgs::Point& next) {
    double dx = next.x - current.x;
    double dy = next.y - current.y;
    double yaw = std::atan2(dy, dx);
    return yawToQuaternion(yaw);
}

// Function to return the static pose pairs
std::vector<std::pair<geometry_msgs::Pose, geometry_msgs::Pose>> getStaticPosePairs(std::vector<geometry_msgs::Pose> poses) {
    std::vector<std::pair<geometry_msgs::Pose, geometry_msgs::Pose>> pose_pairs;

    geometry_msgs::Pose start_pose = poses[0];

    // Loop through each pose and create pose pairs
    for (size_t i = 1; i < poses.size(); ++i) {
        
        // Create the goal pose for the current pair
        geometry_msgs::Pose goal_pose = poses[i];

        // Push the start and goal pair to the vector
        pose_pairs.push_back(std::make_pair(start_pose, goal_pose));

        // Update last_goal_pose for the next iteration
        start_pose = goal_pose;
    }

    return pose_pairs;
}

// Function to plan a path using Move Base's make_plan service
nav_msgs::Path planPathWithMoveBase(const geometry_msgs::Pose& start, const geometry_msgs::Pose& goal, ros::NodeHandle& nh)
{
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

void updateVelocityParams(ros::NodeHandle& nh) {
    nh.setParam("/move_base/TrajectoryPlannerROS/max_vel_x", 0.5);
    nh.setParam("/move_base/TrajectoryPlannerROS/min_vel_x", 0.5);
    nh.setParam("/move_base/TrajectoryPlannerROS/max_vel_theta", 0.25);
}

void updateAccelerationParams(ros::NodeHandle& nh) {
    nh.setParam("/move_base/TrajectoryPlannerROS/acc_lim_x", 0.5);
    nh.setParam("/move_base/TrajectoryPlannerROS/acc_lim_theta", 0.25);
}

// Function to request a path to a goal
void planPathToGoal(const geometry_msgs::Pose& goal, ros::NodeHandle& nh, MoveBaseClient& move_base_client)
{

    move_base_msgs::MoveBaseGoal move_goal;
    move_goal.target_pose.header.frame_id = "map";
    move_goal.target_pose.header.stamp = ros::Time::now();
    move_goal.target_pose.pose = goal;

    // Send the goal to the move_base action server
    move_base_client.sendGoal(move_goal);

    // Wait for the result from MoveBase
    move_base_client.waitForResult(ros::Duration(60.0));

    if (move_base_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Goal reached!");
    else
        ROS_ERROR("Failed to reach the goal.");
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_base_amcl_integration");
    ros::NodeHandle nh;
    
    // Use simulated time
    nh.setParam("use_sim_time", true);
    
    // Create the MoveBase action client
    MoveBaseClient move_base_client("move_base", true);

    // Wait for the action server to be available
    move_base_client.waitForServer();

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

    ROS_INFO("Current pose: %f and %f", current_pose.position.x, current_pose.position.y);

    // Save the paths to a file

    std::string package_path = ros::package::getPath("p3at_plugin");

    if (package_path.empty()) {
        ROS_ERROR("Could not find package 'p3at_plugin'.");
        return 1;
    }

    std::string odom_path = package_path + "/odom.csv";

    // ----------  CREATE AND EXECUTE DIRECT GOALS ------------

    std::vector<geometry_msgs::Pose> poses;

    poses.push_back(current_pose);

    // Loop through each pose data and create pose pairs
    for (auto pose : pose_data)
    {
        // Create the goal pose for the current pair
        geometry_msgs::Pose goal_pose = createPose(pose.x, pose.y, 0.0);
        poses.push_back(goal_pose);
    }
    // Add quaternion orientation to the poses
    for (size_t i = 0; i < poses.size() - 1; ++i)
    {
        poses[i].orientation = calculateOrientation(poses[i].position, poses[i + 1].position);
    }

    // Update velocity and acceleration parameters
    // updateVelocityParams(nh);
    // updateAccelerationParams(nh);

    for (const auto& pose : poses)
    {
        planPathToGoal(pose, nh, move_base_client);
    }

    // ----------  SAVE A PLANNED PATH FOR EXPORT ONLY PURPOSE ------------

    // Get the static pose pairs
    std::vector<std::pair<geometry_msgs::Pose, geometry_msgs::Pose>> static_pose_pairs = getStaticPosePairs(poses);
    std::vector<nav_msgs::Path> paths;
    
    // First, compute all paths and smooth them
    for (const auto& pose_pair : static_pose_pairs)
    {
        const auto& start = pose_pair.first;
        const auto& goal = pose_pair.second;

        // Plan the path with Move Base
        nav_msgs::Path global_path = planPathWithMoveBase(start, goal, nh);

        if (!global_path.poses.empty()) {
           
            paths.push_back(global_path);
        }
        else {
            ROS_ERROR("Failed to plan a path from (%f, %f) to (%f, %f)", start.position.x, start.position.y, goal.position.x, goal.position.y);
        }
    }

    std::string file_path = package_path + "/paths.txt";

    // Open the file and write the paths
    std::ofstream file(file_path);

    if (!file.is_open()) {
        ROS_ERROR("Failed to open file: %s", file_path.c_str());
        return 1;
    }

    for (const auto& path : paths) {
        for (const auto& pose_stamped : path.poses) {
            const auto& pose = pose_stamped.pose;
            file << pose.position.x << " " << pose.position.y << " " << pose.position.z << " "
                 << pose.orientation.x << " " << pose.orientation.y << " " << pose.orientation.z << " " << pose.orientation.w << "\n";
        }
        file << "\n"; // Separate paths with a blank line
    }

    file.close();
    ROS_INFO("Paths saved to %s", file_path.c_str());
    return 0;
}   