#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cmath>
#include <limits>

ros::Publisher pub_combined;
tf::TransformListener* tf_listener;

constexpr float ANGLE_MIN = -M_PI;
constexpr float ANGLE_MAX = M_PI;

float min_range_front = 0.2;
float max_range_front = 6;
float min_range_back  = 0.2;
float max_range_back  = 6;

void transform_and_insert(const sensor_msgs::LaserScan::ConstPtr& scan,
                          std::vector<float>& bins,
                          float angle_increment,
                          const std::string& target_frame,
                          float range_min_filter,
                          float range_max_filter) {
    try {
        tf::StampedTransform transform;
        tf_listener->lookupTransform(target_frame, scan->header.frame_id, scan->header.stamp, transform);

        for (size_t i = 0; i < scan->ranges.size(); ++i) {
            float r = scan->ranges[i];
            if (!std::isfinite(r) || r < range_min_filter || r > range_max_filter)
                continue;

            float angle = scan->angle_min + i * scan->angle_increment;
            tf::Vector3 point(r * cos(angle), r * sin(angle), 0.0);
            tf::Vector3 transformed = transform * point;

            float angle_out = atan2(transformed.y(), transformed.x());
            float range_out = hypot(transformed.x(), transformed.y());

            if (!std::isfinite(range_out) || range_out < range_min_filter || range_out > range_max_filter)
                continue;

            int bin = static_cast<int>((angle_out - ANGLE_MIN) / angle_increment);
            if (bin >= 0 && bin < static_cast<int>(bins.size())) {
                if (std::isinf(bins[bin]) || range_out < bins[bin])
                    bins[bin] = range_out;
            }
        }
    } catch (tf::TransformException& ex) {
        ROS_WARN_STREAM_THROTTLE(2.0, "Transform error: " << ex.what());
    }
}

void callback(const sensor_msgs::LaserScan::ConstPtr& front,
              const sensor_msgs::LaserScan::ConstPtr& back) {
    // Controllo consistenza tra i due scanner
    if (std::abs(front->angle_increment - back->angle_increment) > 1e-6) {
        ROS_WARN_THROTTLE(10, "Laser scans have different angle_increment. Aborting.");
        return;
    }

    float angle_increment = front->angle_increment;
    int total_bins = static_cast<int>((ANGLE_MAX - ANGLE_MIN) / angle_increment);
    std::vector<float> bins(total_bins, std::numeric_limits<float>::infinity());

    transform_and_insert(front, bins, angle_increment, "base_link", min_range_front, max_range_front);
    transform_and_insert(back,  bins, angle_increment, "base_link", min_range_back,  max_range_back);

    sensor_msgs::LaserScan combined;
    combined.header.stamp = front->header.stamp;  // IMPORTANT: usa lo stesso tempo del bag!
    combined.header.frame_id = "base_link";
    combined.angle_min = ANGLE_MIN;
    combined.angle_max = ANGLE_MAX;
    combined.angle_increment = angle_increment;
    combined.range_min = std::min(min_range_front, min_range_back);
    combined.range_max = std::max(max_range_front, max_range_back);
    combined.ranges = bins;

    pub_combined.publish(combined);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "combine_laserscan");
    ros::NodeHandle nh;

    tf_listener = new tf::TransformListener();

    message_filters::Subscriber<sensor_msgs::LaserScan> sub_front(nh, "/scan_front", 1);
    message_filters::Subscriber<sensor_msgs::LaserScan> sub_back(nh, "/scan_back", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan> SyncPolicy;
    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), sub_front, sub_back);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    pub_combined = nh.advertise<sensor_msgs::LaserScan>("/scan_combined", 1);

    ros::spin();
    delete tf_listener;
    return 0;
}
