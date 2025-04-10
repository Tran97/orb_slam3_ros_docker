#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "dynamic_transform_broadcaster_node");
    ros::NodeHandle nh;

    // Transform listener and broadcaster
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    tf2_ros::TransformBroadcaster tfBroadcaster;

    ros::Rate rate(50.0);  // Set update rate to 50 Hz

    while (ros::ok()) {
        try {
            // Get the transform from /world to /camera (from ORB-SLAM3)
            geometry_msgs::TransformStamped world_to_camera;
            world_to_camera = tfBuffer.lookupTransform("world", "camera", ros::Time(0));

            // Get the transform from /base_link to /front_realsense (from the Husky Robot URDF)
            geometry_msgs::TransformStamped base_to_camera;
            base_to_camera = tfBuffer.lookupTransform("base_link", "front_realsense", ros::Time(0));

            // Get the orb transform
            geometry_msgs::TransformStamped orb_transform;
            orb_transform = tfBuffer.lookupTransform("world", "world_orb", ros::Time(0));

            // Add a static transform to account for the orbslam convention:
            tf2::Transform tf_orb_to_base;
            tf_orb_to_base.setOrigin(tf2::Vector3(0.5, 0.2, 0.0));
            tf_orb_to_base.setRotation(tf2::Quaternion(0, 0, 0.7071068, 0.7071068));

            // TODO: WHY ADD THIS STATIC TRANSFORM?

            // Convert geometry_msgs transforms to tf2 transforms
            tf2::Transform tf_world_to_camera, tf_base_to_camera, tf_orb_transform;
            tf2::fromMsg(world_to_camera.transform, tf_world_to_camera);
            tf2::fromMsg(base_to_camera.transform, tf_base_to_camera);
            tf2::fromMsg(orb_transform.transform, tf_orb_transform);

            // Compute the transform from /odom to /base_link
            tf2::Transform tf_odom_to_base = tf_world_to_camera * tf_base_to_camera.inverse() * tf_orb_transform.inverse();


            tf_odom_to_base = tf_odom_to_base * tf_orb_to_base;

            // Convert the resulting transform back to geometry_msgs
            geometry_msgs::TransformStamped odom_to_base;
            odom_to_base.header.stamp = ros::Time::now();
            odom_to_base.header.frame_id = "odom";      // Parent frame
            odom_to_base.child_frame_id = "base_link";  // Child frame
            odom_to_base.transform = tf2::toMsg(tf_odom_to_base);

            // Broadcast the new transform
            tfBroadcaster.sendTransform(odom_to_base);
        } catch (tf2::TransformException &ex) {
            ROS_WARN("Transform not available: %s", ex.what());
        }

        rate.sleep();
    }

    return 0;
}
