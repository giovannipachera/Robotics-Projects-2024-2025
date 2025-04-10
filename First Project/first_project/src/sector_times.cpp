#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "sensor_msgs/NavSatFix.h"
#include "first_project/Sector_time.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

float lat_sec_1, lon_sec_1, lat_sec_2, lon_sec_2, lat_sec_3, lon_sec_3;
ros::Publisher pub;
first_project::Sector_time sector_time;
int current_sector;
ros::Time initial_time;
float initial_speed_kmh;

void callback(const geometry_msgs::PointStampedConstPtr& msg1, const sensor_msgs::NavSatFixConstPtr& msg2)
{
    // Time
    ros::Time current_time = msg1->header.stamp;

    // Input
    float current_speed_kmh = msg1->point.y;
    float current_lat = msg2->latitude;
    float current_lon = msg2->longitude;

    // Custom message variables
    float current_sector_time;
    float current_sector_mean_speed;

    // Transitions between sectors, based on coordinates approximation
    if (current_sector == 0) // Initial state: car is located in sector one
    {
        current_sector = 1;
        initial_time = current_time;
        initial_speed_kmh = current_speed_kmh;
    }
    else if (current_sector == 1 && (current_lat > lat_sec_2 - 0.0005 && current_lat < lat_sec_2 + 0.0005) && (current_lon > lon_sec_2 - 0.0005 && current_lon < lon_sec_2 + 0.0005))
    {
        current_sector = 2;
        initial_time = current_time;
        initial_speed_kmh = current_speed_kmh; 
    }
    else if (current_sector == 2 && (current_lat > lat_sec_3 - 0.0005 && current_lat < lat_sec_3 + 0.0005) && (current_lon > lon_sec_3 - 0.0005 && current_lon < lon_sec_3 + 0.0005))
    {
        current_sector = 3; 
        initial_time = current_time;
        initial_speed_kmh = current_speed_kmh; 
    }
    else if (current_sector == 3 && (current_lat > lat_sec_1 - 0.0005 && current_lat < lat_sec_1 + 0.0005) && (current_lon > lon_sec_1 - 0.0005 && current_lon < lon_sec_1 + 0.0005))
    {
        current_sector = 1; 
        initial_time = current_time;
        initial_speed_kmh = current_speed_kmh; 
    }

    current_sector_time = current_time.toSec() - initial_time.toSec();
    current_sector_mean_speed = (initial_speed_kmh + current_speed_kmh) / 2;

    // Update of custom message
    sector_time.current_sector = current_sector;
    sector_time.current_sector_time = current_sector_time;
    sector_time.current_sector_mean_speed = current_sector_mean_speed;

    // ROS_INFO("current_sector: %d current_sector_time: %f current_sector_mean_speed: %f", sector_time.current_sector, sector_time.current_sector_time, sector_time.current_sector_mean_speed);

    pub.publish(sector_time);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "sector_times");
    ros::NodeHandle nh;

    nh.getParam("/lat_sec_1", lat_sec_1);
    nh.getParam("/lon_sec_1", lon_sec_1);
    nh.getParam("/lat_sec_2", lat_sec_2);
    nh.getParam("/lon_sec_2", lon_sec_2);
    nh.getParam("/lat_sec_3", lat_sec_3);
    nh.getParam("/lon_sec_3", lon_sec_3);

    pub = nh.advertise<first_project::Sector_time>("sector_times", 50);

    message_filters::Subscriber<geometry_msgs::PointStamped> speed_sub(nh, "speedsteer", 50);
    message_filters::Subscriber<sensor_msgs::NavSatFix> gps_sub(nh, "swiftnav/front/gps_pose", 50);

    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PointStamped, sensor_msgs::NavSatFix> MySyncPolicy;

    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(50), speed_sub, gps_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));
    ros::spin();

    return 0;
}