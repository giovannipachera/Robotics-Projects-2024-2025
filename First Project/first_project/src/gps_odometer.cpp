#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"
#include "first_project/GpsToOdom.h"
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <math.h>

ros::Publisher odom_pub;
ros::ServiceClient gps_to_odom_client;
tf::TransformBroadcaster *odom_broadcaster;

// State variables
double x = 0.0, y = 0.0, z = 0.0;
double last_x, last_y, yaw;

// Variables for smoothing filter
const double alpha = 0.1;
double filtered_yaw = 0.0;

bool has_filtered = false;
bool has_last_position = false;
bool ready_to_publish = false;

void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    // Preparing service call
    first_project::GpsToOdom srv;
    srv.request.latitude = msg->latitude;
    srv.request.longitude = msg->longitude;
    srv.request.altitude = msg->altitude;

    // Service call
    if (gps_to_odom_client.call(srv))
    {
        x = srv.response.x;
        y = srv.response.y;
        z = srv.response.z;

        // Yaw calculation
        if (has_last_position)
        {
            double dx = x - last_x;
            double dy = y - last_y;
            yaw = filtered_yaw;

            // Smoothing filter
            if (dx * dx + dy * dy > 0.25)
            {
                double raw_yaw = atan2(dy, dx);

                yaw = has_filtered ? alpha * raw_yaw + (1 - alpha) * filtered_yaw : raw_yaw;
                filtered_yaw = yaw;

                has_filtered = true;
                ready_to_publish = true; // Can start to publish in the right initial direction
            }
        }

        last_x = x;
        last_y = y;
        has_last_position = true;

        if (!ready_to_publish)
            return;

        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(yaw);

        // Publish odometry message
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = msg->header.stamp;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "gps";
        odom_msg.pose.pose.position.x = x;
        odom_msg.pose.pose.position.y = y;
        odom_msg.pose.pose.position.z = z;
        odom_msg.pose.pose.orientation = odom_quat;

        odom_pub.publish(odom_msg);

        // Publish transform TF
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = msg->header.stamp;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "gps";
        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = z;
        odom_trans.transform.rotation = odom_quat;

        odom_broadcaster->sendTransform(odom_trans);
    }
    else
    {
        ROS_ERROR("Failed to call service gps_to_odom");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gps_odometer");
    ros::NodeHandle nh;

    // Odometry message publisher
    odom_pub = nh.advertise<nav_msgs::Odometry>("gps_odom", 10);

    // Transform TF broadcaster
    odom_broadcaster = new tf::TransformBroadcaster();
    
    // Gps data -> Odom converter
    gps_to_odom_client = nh.serviceClient<first_project::GpsToOdom>("gps_to_odom");
    ros::Subscriber gps_sub = nh.subscribe("/swiftnav/front/gps_pose", 10, gpsCallback);

    ros::spin();

    delete odom_broadcaster;
    return 0;
}
