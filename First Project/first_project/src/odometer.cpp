#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <math.h>

// Vehicle parameters
const double WHEELBASE = 1.765;			// distance between front/back wheels (m)
const double STEERING_FACTOR = 32.0;	// steer degrees to front wheels angle
const double DEG_TO_RAD = M_PI / 180.0;	// degrees to radiants conv

// actual state
double x = 0.0, y = 0.0, theta = 0.0;
ros::Time last_time;

ros::Publisher odom_pub;
tf::TransformBroadcaster *odom_broadcaster;

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
	double steering_angle_rad = (steer_deg / STEERING_FACTOR) * DEG_TO_RAD;
	
	// Update state
	double delta_x = speed_ms * cos(theta) * dt;
	double delta_y = speed_ms * sin(theta) * dt;
	double delta_theta = (speed_ms / WHEELBASE) * tan(steering_angle_rad) * dt;
	
	x += delta_x;
	y += delta_y;
	theta += delta_theta;
	
	// Odometry message
	nav_msgs::Odometry odom;
	odom.header.stamp = current_time;
	odom.header.frame_id = "odom";
	odom.child_frame_id = "vehicle";
	
	odom.pose.pose.position.x = x;
	odom.pose.pose.position.y = y;
	odom.pose.pose.position.z = 0.0;
	
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);
	odom.pose.pose.orientation = odom_quat;
	
	odom.twist.twist.linear.x = speed_ms;
	odom.twist.twist.angular.z = (speed_ms / WHEELBASE) * tan(steering_angle_rad);
	
	odom_pub.publish(odom);
	
	// Publish transform TF
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = current_time;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "vehicle";
	
	odom_trans.transform.translation.x = x;
	odom_trans.transform.translation.y = y;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = odom_quat;
	
	odom_broadcaster->sendTransform(odom_trans);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "odometer");
	ros::NodeHandle nh;
	
	odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
	odom_broadcaster = new tf::TransformBroadcaster();
	
	ros::Subscriber sub = nh.subscribe("speedsteer", 1000, speedSteerCallback);
	
	ros::spin();
	
	delete odom_broadcaster;
	return 0;
}