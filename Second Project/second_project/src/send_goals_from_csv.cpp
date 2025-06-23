#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <tf/tf.h>
#include <ros/package.h>
#include <cmath>

int main(int argc, char** argv) {
    ros::init(argc, argv, "csv_goal_sender");
    ros::NodeHandle nh;

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
    ROS_INFO("Waiting for move_base action server...");
    ac.waitForServer();
    ROS_INFO("Connected to move_base.");

    std::ifstream file(ros::package::getPath("second_project") + "/csv/goals.csv");
    if (!file.is_open()) {
        ROS_ERROR("Failed to open goals.csv");
        return 1;
    }

    std::string line;
    std::getline(file, line);
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string x_str, y_str, theta_str;
        std::getline(ss, x_str, ',');
        std::getline(ss, y_str, ',');
        std::getline(ss, theta_str, ',');

        double x = std::stod(x_str);
        double y = std::stod(y_str);
        double theta_deg = std::stod(theta_str);
        double theta_rad = theta_deg * M_PI / 180.0;

        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = x;
        goal.target_pose.pose.position.y = y;
        goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta_rad);

        ROS_INFO("Sending goal: (%.2f, %.2f, %.2f)", x, y, theta_deg);
        ac.sendGoal(goal);

        ac.waitForResult(ros::Duration(60.0));
        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Goal reached.");
        } else {
            ROS_WARN("Failed to reach goal.");
        }

        ros::Duration(2.0).sleep();  // pausa tra i goal
    }

    ROS_INFO("All goals processed.");
    return 0;
}
