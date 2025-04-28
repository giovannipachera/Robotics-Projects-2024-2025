#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <math.h>

ros::Publisher odom_pub;
tf::TransformBroadcaster *odom_broadcaster;

// Vehicle parameters
const double WHEELBASE = 1.765;         // Distance between front and rear wheels (m)
const double STEERING_FACTOR = 32.0;    // Steer degrees to front wheels angle
const double DEG_TO_RAD = M_PI / 180.0; // Degrees to radians conversion

// State variables
double x = 0.0, y = 0.0;
double theta = 1.49; // set to initial orientation
ros::Time last_time;

// Bias correction
double STEERING_BIAS;

// Sequence counter for bias correction
int sequence = 0;

void speedSteerCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    // Time
    ros::Time current_time = msg->header.stamp;
    if (last_time.isZero()) 
    {
        last_time = current_time;
        return;
    }
    double dt = (current_time - last_time).toSec();
    last_time = current_time;

    // Input
	double steer_deg = msg->point.x;
    double speed_kmh = msg->point.y;
    double speed_ms = speed_kmh / 3.6;

    // Steering bias correction
    if (sequence > 320)
    {
        steer_deg += STEERING_BIAS;
    }

    double steering_angle_rad = (steer_deg / STEERING_FACTOR) * DEG_TO_RAD;

    // Compute angular velocity
    double omega = (speed_ms / WHEELBASE) * tan(steering_angle_rad);
    double theta_new = theta + omega * dt;

    // Update state
    if (fabs(omega) > 0.01) // Exact integration
    { 
        x += (speed_ms / omega) * (sin(theta_new) - sin(theta));
        y += -(speed_ms / omega) * (cos(theta_new) - cos(theta));
    } 
    else // Runge-Kutta integration (for omega ~ 0)
    {
        x += speed_ms * dt * cos(theta + 0.5 * omega * dt);
        y += speed_ms * dt * sin(theta + 0.5 * omega * dt);
    }

    theta = theta_new;
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

    // Publish odometry message
    nav_msgs::Odometry odom;
    odom.header.stamp = msg->header.stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "vehicle";
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.twist.twist.linear.x = speed_ms;
    odom.twist.twist.angular.z = omega;
    odom.pose.pose.orientation = odom_quat;

    odom_pub.publish(odom);

    // Publish transform TF
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = msg->header.stamp;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "vehicle";
    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    odom_broadcaster->sendTransform(odom_trans);

	sequence++;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odometer");
    ros::NodeHandle nh;

    // Steering bias correction parameter
    nh.param("steering_bias", STEERING_BIAS, 0.0);

    // Odometry message publisher
    odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);

    // Transform TF broadcaster
    odom_broadcaster = new tf::TransformBroadcaster();

    // Speedsteer data subscriber
    ros::Subscriber sub = nh.subscribe("speedsteer", 1000, speedSteerCallback);

    ros::spin();

    delete odom_broadcaster;
    return 0;
}
