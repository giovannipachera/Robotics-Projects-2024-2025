#include "ros/ros.h"
#include "first_project/GpsToOdom.h"
#include <cmath>

#define DEG2RAD M_PI / 180.0

double lat_ref, lon_ref, alt_ref;

// Conversion
const double a = 6378137.0;       // semi-asse maggiore WGS84
const double f = 1.0 / 298.257223563;
const double b = a * (1 - f);
const double e_sq = f * (2 - f);

void gpsToECEF(double lat, double lon, double alt, double& x, double& y, double& z) {
    lat *= DEG2RAD;
    lon *= DEG2RAD;

    double N = a / sqrt(1 - e_sq * sin(lat) * sin(lat));
    x = (N + alt) * cos(lat) * cos(lon);
    y = (N + alt) * cos(lat) * sin(lon);
    z = ((1 - e_sq) * N + alt) * sin(lat);
}

void ecefToENU(double x, double y, double z,
               double x0, double y0, double z0,
               double lat0, double lon0,
               double& east, double& north, double& up) {
    lat0 *= DEG2RAD;
    lon0 *= DEG2RAD;

    double dx = x - x0;
    double dy = y - y0;
    double dz = z - z0;

    east  = -sin(lon0) * dx + cos(lon0) * dy;
    north = -sin(lat0) * cos(lon0) * dx - sin(lat0) * sin(lon0) * dy + cos(lat0) * dz;
    up    =  cos(lat0) * cos(lon0) * dx + cos(lat0) * sin(lon0) * dy + sin(lat0) * dz;
}

bool convertCallback(first_project::GpsToOdom::Request &req,
                     first_project::GpsToOdom::Response &res)
{
    double x, y, z;
    gpsToECEF(req.latitude, req.longitude, req.altitude, x, y, z);

    double x0, y0, z0;
    gpsToECEF(lat_ref, lon_ref, alt_ref, x0, y0, z0);

    ecefToENU(x, y, z, x0, y0, z0, lat_ref, lon_ref, res.x, res.y, res.z);

    ROS_INFO("GPS: [%.6f, %.6f, %.2f] -> ENU: [%.2f, %.2f, %.2f]",
             req.latitude, req.longitude, req.altitude,
             res.x, res.y, res.z);

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gps_to_odom_server");
    ros::NodeHandle nh;

    nh.getParam("lat_ref", lat_ref);
    nh.getParam("lon_ref", lon_ref);
    nh.getParam("alt_ref", alt_ref);

    ros::ServiceServer service = nh.advertiseService("gps_to_odom", convertCallback);
    ROS_INFO("Service 'gps_to_odom' ready.");
    ros::spin();

    return 0;
}