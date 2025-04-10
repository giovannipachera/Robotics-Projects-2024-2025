#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"
#include "first_project/GpsToOdom.h"

ros::Publisher odom_pub;
ros::ServiceClient gps_to_odom_client;

void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    // Prepara la richiesta per il servizio
    first_project::GpsToOdom srv;
    srv.request.latitude = msg->latitude;
    srv.request.longitude = msg->longitude;
    srv.request.altitude = msg->altitude;

    if (gps_to_odom_client.call(srv))
    {
        // Se la chiamata ha successo, prepara il messaggio Odometry
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = ros::Time::now();
        odom_msg.header.frame_id = "odom";

        // Riempie i dati con la risposta del servizio
        odom_msg.pose.pose.position.x = srv.response.x;
        odom_msg.pose.pose.position.y = srv.response.y;
        odom_msg.pose.pose.position.z = srv.response.z;

        // Puoi anche azzerare l'orientamento inizialmente
        odom_msg.pose.pose.orientation.x = 0.0;
        odom_msg.pose.pose.orientation.y = 0.0;
        odom_msg.pose.pose.orientation.z = 0.0;
        odom_msg.pose.pose.orientation.w = 1.0;

        odom_pub.publish(odom_msg);
    }
    else
    {
        ROS_ERROR("Failed to call service gps_to_odom_server");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gps_odometer");
    ros::NodeHandle nh;

    // Publisher per il messaggio Odometry
    odom_pub = nh.advertise<nav_msgs::Odometry>("gps_odom", 10);

    // Client per il servizio di conversione
    gps_to_odom_client = nh.serviceClient<first_project::GpsToOdom>("gps_to_odom_server");

    // Subscriber ai dati GPS
    ros::Subscriber gps_sub = nh.subscribe("gps_data", 10, gpsCallback);

    ros::spin();

    return 0;
}
