/**
 * @file localizer.h
 * @author amsl
 */
#ifndef __NODE_EDGE_LOCALIZER_LOCALIZER_H
#define __NODE_EDGE_LOCALIZER_LOCALIZER_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Dense>

namespace node_edge_localizer
{
struct Pose
{
    // x, y, z
    Eigen::Vector3d position_;
    double yaw_;
};

class Localizer
{
public:
    Localizer(void);

    void odom_callback(const nav_msgs::OdometryConstPtr& msg);
    void initialize(void);
    nav_msgs::Odometry convert_pose_to_msg(const Pose& p);
    void process(void);
protected:
    ros::NodeHandle nh_;
    ros::NodeHandle local_nh_;
    ros::Publisher estimated_pose_pub_;
    ros::Subscriber odom_sub_;
    Pose estimated_pose_;
};
}// namespace node_edge_localizer

#endif// __NODE_EDGE_LOCALIZER_LOCALZER_H