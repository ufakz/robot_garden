#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv) {
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "actionlib");

  // Tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while (!ac.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  

  move_base_msgs::MoveBaseGoal goal;

  // Set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  float goals[2][3] = {{-2.0, 0.0, 1.57}, {0.0, 0.0, 1.57}};
  for (int i = 0; i < 2; i++) {
    // Define a position and orientation for the robot to reach
    goal.target_pose.pose.position.x = goals[i][0];
    goal.target_pose.pose.position.y = goals[i][1];
    goal.target_pose.pose.orientation.z = goals[i][2];

    // Send the goal to the robot
    ROS_INFO("Sending goal %d", i + 1);
    ac.sendGoal(goal);

    // Wait an infinite time for the results
    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("Hooray, the robot reached goal %d!", i + 1);
    } else {
      ROS_INFO("The robot failed to reach goal %d", i + 1);
    }

    ros::Duration(5.0).sleep(); // Pause for 5 seconds between goals
  }

  return 0;
}