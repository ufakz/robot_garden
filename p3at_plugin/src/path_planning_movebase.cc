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
#include "waypoints.h"

class RobotNavigator {

public:
    RobotNavigator();
    void navigate();

private:
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber odom_sub_;

    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
    std::unique_ptr<MoveBaseClient> ac_;

    std::vector<nav_msgs::Path> paths_;
    size_t current_path_index_;

    double current_x_;
    double current_y_;
    double current_yaw_;
    bool pose_initialized_;

    geometry_msgs::Quaternion calculateOrientation(const geometry_msgs::Point& current, const geometry_msgs::Point& next);
    nav_msgs::Path samplePath(const nav_msgs::Path& original_path, size_t NUM_SAMPLES);
    nav_msgs::Path planPathWithMoveBase(ros::NodeHandle& nh, const geometry_msgs::Pose& start, const geometry_msgs::Pose& goal);
    geometry_msgs::Pose getCurrentPose();

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void planPathToGoal(const geometry_msgs::Pose& goal);
    void executeTrajectory(const nav_msgs::Path& path, ros::Publisher& cmd_vel_pub);
    bool goToGoals();
};

RobotNavigator::RobotNavigator() : 
    current_path_index_(0), 
    pose_initialized_(false) {
    
    nh_.setParam("use_sim_time", true);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    odom_sub_ = nh_.subscribe("/odom", 1000, &RobotNavigator::odomCallback, this);

    ac_ = std::make_unique<MoveBaseClient>("move_base", true);

    // Wait for the action server to come up
    while(!ac_->waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    // Create waypoints vector
    std::vector<std::pair<float, float>> points;
    for (const auto& waypoint : GARDEN_1_WAYPOINTS) {
        points.push_back({waypoint.x, waypoint.y});
    }

    geometry_msgs::Pose start_pose = getCurrentPose();
    for (const auto& location : points) {
        geometry_msgs::Pose goal_pose;
        goal_pose.position.x = location.first;
        goal_pose.position.y = location.second;
        goal_pose.position.z = 0.0;

        nav_msgs::Path global_path = planPathWithMoveBase(nh_, start_pose, goal_pose);
        
        if (!global_path.poses.empty()) {
            paths_.push_back(samplePath(global_path, 5)); // Provide the second argument
        }

        start_pose = goal_pose;
    }
}

geometry_msgs::Quaternion RobotNavigator::calculateOrientation(const geometry_msgs::Point& current, 
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

nav_msgs::Path RobotNavigator::samplePath(const nav_msgs::Path& original_path, size_t NUM_SAMPLES = 5) {
    nav_msgs::Path sampled_path;
    sampled_path.header = original_path.header;
    
    if (original_path.poses.size() <= NUM_SAMPLES) {
        return original_path;
    }

    double stride = static_cast<double>(original_path.poses.size() - 1) / (NUM_SAMPLES - 1);
    
    for (size_t i = 0; i < NUM_SAMPLES; i++) {
        size_t current_index = static_cast<size_t>(i * stride);
        if (i == NUM_SAMPLES - 1) {
            current_index = original_path.poses.size() - 1;
        }

        geometry_msgs::PoseStamped pose = original_path.poses[current_index];
        
        // Calculate orientation
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
            pose.pose.orientation = original_path.poses.back().pose.orientation;
        }
        
        sampled_path.poses.push_back(pose);
    }

    return sampled_path;
}

nav_msgs::Path RobotNavigator::planPathWithMoveBase(ros::NodeHandle& nh, 
                                    const geometry_msgs::Pose& start, 
                                    const geometry_msgs::Pose& goal) {
    ros::ServiceClient make_plan_client = nh.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");
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

geometry_msgs::Pose RobotNavigator::getCurrentPose() {
    tf::TransformListener listener;
    tf::StampedTransform transform;
    geometry_msgs::Pose current_pose;

    try {
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
    catch (tf::TransformException &ex) {
        ROS_ERROR("Transform error: %s", ex.what());
    }

    return current_pose;
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

void RobotNavigator::planPathToGoal(const geometry_msgs::Pose& goal) {
    
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

void RobotNavigator::executeTrajectory(const nav_msgs::Path& path, ros::Publisher& cmd_vel_pub){
        
    if (path.poses.empty()) {
        ROS_ERROR("Path is empty, cannot execute trajectory.");
        return;
    }

    ros::Rate rate(10);
    for (const auto& pose_stamped : path.poses) {
        geometry_msgs::Pose pose = pose_stamped.pose;
        planPathToGoal(pose);
        rate.sleep();
    }

}

bool RobotNavigator::goToGoals() {
        
    if (paths_.empty()) {
        ROS_WARN("No paths available for navigation");
        return true;  // Return true since there's nothing to do
    }

    if (current_path_index_ >= paths_.size()) {
        ROS_INFO("Completed all paths!");
        return true;
    }

    const nav_msgs::Path& current_path = paths_[current_path_index_];

    executeTrajectory(current_path, cmd_vel_pub_);
    current_path_index_++;

    return false;  // Still have more goals to process
}


void RobotNavigator::navigate() {
    ros::Rate rate(10);

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

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "path_planning_movebase");
    RobotNavigator navigator;
    navigator.navigate();
    return 0;
}