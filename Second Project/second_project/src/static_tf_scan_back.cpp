#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "static_tf_scan_back");
    ros::NodeHandle nh;

    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped static_transform;

    static_transform.header.stamp = ros::Time::now();  // va bene, ma non conta in static TF
    static_transform.header.frame_id = "base_link";
    static_transform.child_frame_id = "sick_back";

    // Posizione del laser posteriore
    static_transform.transform.translation.x = -0.3;
    static_transform.transform.translation.y = 0.0;
    static_transform.transform.translation.z = -0.115;

    // Quaternion iniziale del front laser (copiato da tf_static)
    tf2::Quaternion q_front(
        -0.9999993658637698,
         0.0007963264582434141,
         0.0007963264582434141,
         6.341362302272584e-07
    );

    // Rotazione di 180° attorno a Z per ottenere il retro
    tf2::Quaternion q_rot;
    q_rot.setRPY(0, 0, M_PI);

    tf2::Quaternion q_back = q_rot * q_front;
    q_back.normalize();  // 👈 sempre buona norma con composizioni

    static_transform.transform.rotation.x = q_back.x();
    static_transform.transform.rotation.y = q_back.y();
    static_transform.transform.rotation.z = q_back.z();
    static_transform.transform.rotation.w = q_back.w();

    static_broadcaster.sendTransform(static_transform);
    ros::spin();
    return 0;
}
