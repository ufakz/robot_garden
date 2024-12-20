#include <ros/ros.h>
#include <ros/package.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <fstream>
#include <cmath>
#include <vector>
#include "waypoints.h"

class RobotNavigator {
public:
    RobotNavigator();
    void navigate();

private:
    // Type alias for move base action client
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

    // Nested structs and data types
    struct PoseData {
        double x, y, z;
    };

    // ROS node handle
    ros::NodeHandle nh_;
    
    // Move base action client
    MoveBaseClient move_base_client_;

    // Transform listener for getting robot pose
    tf::TransformListener listener_;

    // Utility methods
    geometry_msgs::Quaternion yawToQuaternion(double yaw);
    geometry_msgs::Pose createPose(double x, double y, double z);
    geometry_msgs::Quaternion calculateOrientation(const geometry_msgs::Point& current,
                                                   const geometry_msgs::Point& next);
    
    geometry_msgs::Pose getCurrentPose();
    nav_msgs::Path planPathWithMoveBase(const geometry_msgs::Pose& start, 
                                        const geometry_msgs::Pose& goal);
    void planPathToGoal(const geometry_msgs::Pose& goal);
    void savePaths(const std::vector<nav_msgs::Path>& paths);

    std::vector<std::pair<geometry_msgs::Pose, geometry_msgs::Pose>> 
    getStaticPosePairs(std::vector<geometry_msgs::Pose>& poses);
};

RobotNavigator::RobotNavigator() 
    : move_base_client_("move_base", true) {
    // Set simulated time
    nh_.setParam("use_sim_time", true);

    // Wait for move base action server
    move_base_client_.waitForServer();
}

geometry_msgs::Quaternion RobotNavigator::yawToQuaternion(double yaw) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(yaw / 2.0);
    quaternion.w = cos(yaw / 2.0);
    return quaternion;
}

geometry_msgs::Pose RobotNavigator::createPose(double x, double y, double z) {
    geometry_msgs::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    return pose;
}

geometry_msgs::Quaternion RobotNavigator::calculateOrientation(
    const geometry_msgs::Point& current, const geometry_msgs::Point& next) {
    double dx = next.x - current.x;
    double dy = next.y - current.y;
    double yaw = std::atan2(dy, dx);
    return yawToQuaternion(yaw);
}

geometry_msgs::Pose RobotNavigator::getCurrentPose() {
    geometry_msgs::Pose current_pose;
    tf::StampedTransform transform;

    try {
        listener_.waitForTransform("map", "base_footprint", ros::Time(0), ros::Duration(10.0));
        listener_.lookupTransform("map", "base_footprint", ros::Time(0), transform);

        current_pose.position.x = transform.getOrigin().x();
        current_pose.position.y = transform.getOrigin().y();
        current_pose.position.z = transform.getOrigin().z();
        current_pose.orientation.x = transform.getRotation().x();
        current_pose.orientation.y = transform.getRotation().y();
        current_pose.orientation.z = transform.getRotation().z();
        current_pose.orientation.w = transform.getRotation().w();
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }

    return current_pose;
}

std::vector<std::pair<geometry_msgs::Pose, geometry_msgs::Pose>> 
RobotNavigator::getStaticPosePairs(std::vector<geometry_msgs::Pose>& poses) {
    std::vector<std::pair<geometry_msgs::Pose, geometry_msgs::Pose>> pose_pairs;
    geometry_msgs::Pose start_pose = poses[0];

    for (size_t i = 1; i < poses.size(); ++i) {
        geometry_msgs::Pose goal_pose = poses[i];
        pose_pairs.push_back(std::make_pair(start_pose, goal_pose));
        start_pose = goal_pose;
    }

    return pose_pairs;
}

nav_msgs::Path RobotNavigator::planPathWithMoveBase(
    const geometry_msgs::Pose& start, const geometry_msgs::Pose& goal) {
    ros::ServiceClient make_plan_client = nh_.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");
    make_plan_client.waitForExistence();

    nav_msgs::GetPlan srv;
    srv.request.start.header.frame_id = "map";
    srv.request.start.header.stamp = ros::Time::now();
    srv.request.start.pose = start;

    srv.request.goal.header.frame_id = "map";
    srv.request.goal.header.stamp = ros::Time::now();
    srv.request.goal.pose = goal;

    srv.request.tolerance = 0.0;

    nav_msgs::Path planned_path;
    if (make_plan_client.call(srv)) {
        ROS_INFO("Received plan with %lu waypoints", srv.response.plan.poses.size());
        planned_path = srv.response.plan;
    } else {
        ROS_ERROR("Failed to call service make_plan");
    }

    return planned_path;
}

void RobotNavigator::planPathToGoal(const geometry_msgs::Pose& goal) {
    move_base_msgs::MoveBaseGoal move_goal;
    move_goal.target_pose.header.frame_id = "map";
    move_goal.target_pose.header.stamp = ros::Time::now();
    move_goal.target_pose.pose = goal;

    move_base_client_.sendGoal(move_goal);
    move_base_client_.waitForResult(ros::Duration(60.0));

    if (move_base_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Goal reached!");
    else
        ROS_ERROR("Failed to reach the goal.");
}

void RobotNavigator::savePaths(const std::vector<nav_msgs::Path>& paths) {
    std::string package_path = ros::package::getPath("p3at_plugin");
    if (package_path.empty()) {
        ROS_ERROR("Could not find package 'p3at_plugin'.");
        return;
    }

    std::string file_path = package_path + "/paths.txt";
    std::ofstream file(file_path);

    if (!file.is_open()) {
        ROS_ERROR("Failed to open file: %s", file_path.c_str());
        return;
    }

    for (const auto& path : paths) {
        for (const auto& pose_stamped : path.poses) {
            const auto& pose = pose_stamped.pose;
            file << pose.position.x << " " << pose.position.y << " " << pose.position.z << " "
                 << pose.orientation.x << " " << pose.orientation.y << " " 
                 << pose.orientation.z << " " << pose.orientation.w << "\n";
        }
        file << "\n"; // Separate paths with a blank line
    }

    file.close();
    ROS_INFO("Paths saved to %s", file_path.c_str());
}

void RobotNavigator::navigate() {
    // Get the robot's current pose
    geometry_msgs::Pose current_pose = getCurrentPose();
    ROS_INFO("Current pose: %f and %f", current_pose.position.x, current_pose.position.y);

    // Prepare poses vector
    std::vector<geometry_msgs::Pose> poses;
    poses.push_back(current_pose);

    // Create goal poses
    for (auto pose : GARDEN_1_WAYPOINTS) {
        geometry_msgs::Pose goal_pose = createPose(pose.x, pose.y, pose.z);
        poses.push_back(goal_pose);
    }

    // Add quaternion orientation to the poses
    for (size_t i = 0; i < poses.size() - 1; ++i) {
        poses[i].orientation = calculateOrientation(poses[i].position, poses[i + 1].position);
    }

    // Navigate to each pose
    for (const auto& pose : poses) {
        planPathToGoal(pose);
    }

    // Compute static pose pairs for path planning and saving
    std::vector<std::pair<geometry_msgs::Pose, geometry_msgs::Pose>> 
        static_pose_pairs = getStaticPosePairs(poses);
    std::vector<nav_msgs::Path> paths;
    
    // Compute paths for saving
    for (const auto& pose_pair : static_pose_pairs) {
        const auto& start = pose_pair.first;
        const auto& goal = pose_pair.second;

        nav_msgs::Path global_path = planPathWithMoveBase(start, goal);
        if (!global_path.poses.empty()) {
            paths.push_back(global_path);
        } else {
            ROS_ERROR("Failed to plan a path from (%f, %f) to (%f, %f)", 
                      start.position.x, start.position.y, 
                      goal.position.x, goal.position.y);
        }
    }

    // Save computed paths
    savePaths(paths);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "waypoint_following_movebase");
    RobotNavigator navigator;
    navigator.navigate();
    return 0;
}