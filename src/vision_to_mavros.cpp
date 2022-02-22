#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <mavros_msgs/LandingTarget.h>

#include <cstring>

void load_param(const ros::NodeHandle &nh, const std::string &param_name, std::string &dest) {
    if (nh.getParam(param_name, dest)) {
        ROS_INFO("Get '%s' parameter: %s", param_name.c_str(), dest.c_str());
    } else {
        ROS_WARN("Using default '%s': %s", param_name.c_str(), dest.c_str());
    }
}

void load_param(const ros::NodeHandle &nh, const std::string &param_name, double &dest) {
    if (nh.getParam(param_name, dest)) {
        ROS_INFO("Get '%s' parameter: %f", param_name.c_str(), dest);
    } else {
        ROS_WARN("Using default '%s': %f", param_name.c_str(), dest);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "vision_to_mavros");
    ros::NodeHandle node("~");

    ros::Publisher camera_pose_publisher = node.advertise<geometry_msgs::PoseStamped>("vision_pose", 10);
    ros::Publisher body_path_publisher = node.advertise<nav_msgs::Path>("body_frame/path", 1);

    tf::TransformListener tf_listener;
    tf::StampedTransform transform;

    geometry_msgs::PoseStamped msg_body_pose;
    nav_msgs::Path body_path;

    std::string target_frame_id = "/camera_odom_frame";
    std::string source_frame_id = "/camera_link";

    double output_rate = 20, roll_cam = 0, pitch_cam = 0, yaw_cam = 1.5707963, gamma_world = -1.5707963;

    // The frame in which we find the transform into, the original "world" frame
    load_param(node, "target_frame_id", target_frame_id);

    // The frame for which we find the transform to target_frame_id, the original "camera" frame
    load_param(node, "source_frame_id", source_frame_id);

    // The rate at which we wish to publish final pose data
    load_param(node, "output_rate", output_rate);

    // The rotation around z axis between original world frame and target world frame, assuming the z-axis needs not to be changed.
    // In this case, target world frame has y forward, x to the right and z upwards (ENU as ROS dictates)
    load_param(node, "gamma_world", gamma_world);

    // The roll angle around camera's own axis to align with body frame
    load_param(node, "roll_cam", roll_cam);

    // The pitch angle around camera's own axis to align with body frame
    load_param(node, "pitch_cam", pitch_cam);

    // The yaw angle around camera's own axis to align with body frame
    load_param(node, "yaw_cam", yaw_cam);

    // Variables for precision landing (optional)
    bool enable_precland = false;
    std::string precland_target_frame_id = "/landing_target";
    std::string precland_camera_frame_id = "/camera_fisheye2_optical_frame";

    ros::Publisher precland_msg_publisher;

    if (node.getParam("enable_precland", enable_precland)) {
        ROS_INFO("Precision landing: %s", enable_precland ? "enabled" : "disabled");
    } else {
        ROS_INFO("Precision landing disabled by default");
    }

    if (enable_precland) {
        // The frame of the landing target in the camera frame
        load_param(node, "precland_camera_frame_id", precland_camera_frame_id);
        load_param(node, "precland_target_frame_id", precland_target_frame_id);
        precland_msg_publisher = node.advertise<mavros_msgs::LandingTarget>("landing_raw", 10);
    }

    // Wait for the first transform to become available.
    tf_listener.waitForTransform(target_frame_id, source_frame_id, ros::Time::now(), ros::Duration(3.0));

    ros::Time last_tf_time = ros::Time::now();
    ros::Time last_precland_tf_time = ros::Time::now();

    // Limit the rate of publishing data, otherwise the other telemetry port might be flooded
    ros::Rate rate(output_rate);

    while (node.ok()) {
        // For tf, Time(0) means "the latest available" transform in the buffer.
        ros::Time now = ros::Time(0);

        // Publish vision_position_estimate message if transform is available
        try {
            // lookupTransform(frame_2, frame_1, at_this_time, this_transform)
            //    will give the transform from frame_1 to frame_2
            tf_listener.lookupTransform(target_frame_id, source_frame_id, now, transform);

            // Only publish pose messages when we have new transform data.
            if (last_tf_time < transform.stamp_) {
                last_tf_time = transform.stamp_;

                static tf::Vector3 position_orig, position_body;

                static tf::Quaternion quat_cam, quat_cam_to_body_x, quat_cam_to_body_y, quat_cam_to_body_z, quat_rot_z, quat_body;

                // 1) Rotation from original world frame to world frame with y forward.
                // See the full rotation matrix at https://en.wikipedia.org/wiki/Rotation_matrix#Basic_rotations
                position_orig = transform.getOrigin();

                position_body.setX(cos(gamma_world) * position_orig.getX() + sin(gamma_world) * position_orig.getY());
                position_body.setY(-sin(gamma_world) * position_orig.getX() + cos(gamma_world) * position_orig.getY());
                position_body.setZ(position_orig.getZ());

                // 2) Rotation from camera to body frame.
                quat_cam = transform.getRotation();

                quat_cam_to_body_x = tf::createQuaternionFromRPY(roll_cam, 0, 0);
                quat_cam_to_body_y = tf::createQuaternionFromRPY(0, pitch_cam, 0);
                quat_cam_to_body_z = tf::createQuaternionFromRPY(0, 0, yaw_cam);

                // 3) Rotate body frame 90 degree (align body x with world y at launch)
                quat_rot_z = tf::createQuaternionFromRPY(0, 0, -gamma_world);

                quat_body = quat_rot_z * quat_cam * quat_cam_to_body_x * quat_cam_to_body_y * quat_cam_to_body_z;
                quat_body.normalize();

                // Create PoseStamped message to be sent
                msg_body_pose.header.stamp = transform.stamp_;
                msg_body_pose.header.frame_id = transform.frame_id_;
                msg_body_pose.pose.position.x = position_body.getX();
                msg_body_pose.pose.position.y = position_body.getY();
                msg_body_pose.pose.position.z = position_body.getZ();
                msg_body_pose.pose.orientation.x = quat_body.getX();
                msg_body_pose.pose.orientation.y = quat_body.getY();
                msg_body_pose.pose.orientation.z = quat_body.getZ();
                msg_body_pose.pose.orientation.w = quat_body.getW();

                // Publish pose of body frame in world frame
                camera_pose_publisher.publish(msg_body_pose);

                // Publish trajectory path for visualization
                body_path.header.stamp = msg_body_pose.header.stamp;
                body_path.header.frame_id = msg_body_pose.header.frame_id;
                body_path.poses.push_back(msg_body_pose);
                body_path_publisher.publish(body_path);
            }
        }
        catch (tf::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
        }

        // Publish landing_target message if option is enabled and transform is available
        if (enable_precland) {
            if (tf_listener.canTransform(precland_camera_frame_id, precland_target_frame_id, now)) {
                // lookupTransform(frame_2, frame_1, at_this_time, this_transform)
                //    will give the transform from frame_1 to frame_2
                tf_listener.lookupTransform(precland_camera_frame_id, precland_target_frame_id, now, transform);

                // Only publish when we have new data
                if (last_precland_tf_time < transform.stamp_) {
                    last_precland_tf_time = transform.stamp_;

                    mavros_msgs::LandingTarget msg_landing_target;

                    // Set up the landing target message according to the relative protocol: https://mavlink.io/en/services/landing_target.html#camera_image_relative
                    msg_landing_target.header.frame_id = transform.frame_id_;
                    msg_landing_target.header.stamp = transform.stamp_;
                    msg_landing_target.target_num = 0;
                    msg_landing_target.frame = mavros_msgs::LandingTarget::LOCAL_NED;
                    msg_landing_target.type = mavros_msgs::LandingTarget::VISION_FIDUCIAL;

                    msg_landing_target.angle[0] = static_cast<float>(std::atan(
                            transform.getOrigin().getX() / transform.getOrigin().getZ()));
                    msg_landing_target.angle[1] = static_cast<float>(std::atan(
                            transform.getOrigin().getY() / transform.getOrigin().getZ()));
                    msg_landing_target.distance = static_cast<float>(transform.getOrigin().length());

                    precland_msg_publisher.publish(msg_landing_target);
                    ROS_INFO("Landing target detected");
                }
            }
        }

        rate.sleep();
    }
    return 0;
}