#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <cmath>

tf::TransformBroadcaster* odom_broadcaster;
ros::Time last_stamp;
geometry_msgs::Pose last_pose;

bool pose_changed(const geometry_msgs::Pose& a, const geometry_msgs::Pose& b, double tol = 1e-4) {
    return std::fabs(a.position.x - b.position.x) > tol ||
           std::fabs(a.position.y - b.position.y) > tol ||
           std::fabs(a.orientation.z - b.orientation.z) > tol ||
           std::fabs(a.orientation.w - b.orientation.w) > tol;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    if (!std::isfinite(msg->pose.pose.position.x) ||
        !std::isfinite(msg->pose.pose.position.y)) {
        ROS_WARN_THROTTLE(5, "Non-finite odometry, skipping TF.");
        return;
    }

    // Salta se timestamp identico al precedente (evita TF_REPEATED_DATA)
    if (msg->header.stamp == last_stamp) return;
    last_stamp = msg->header.stamp;

    // Salta se la pose non è cambiata
    if (!pose_changed(msg->pose.pose, last_pose)) return;
    last_pose = msg->pose.pose;

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        0.0
    ));

    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );
    transform.setRotation(q);

    odom_broadcaster->sendTransform(
        tf::StampedTransform(transform, msg->header.stamp, "odom", "base_link")
    );
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "odom_to_tf");
    ros::NodeHandle nh;

    odom_broadcaster = new tf::TransformBroadcaster();
    ros::Subscriber sub = nh.subscribe("/odometry", 10, odomCallback);

    ros::spin();
    delete odom_broadcaster;
    return 0;
}
