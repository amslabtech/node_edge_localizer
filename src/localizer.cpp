/**
 * @file localizer.cpp 
 * @author amsl 
 */

#include "node_edge_localizer/localizer.h"

namespace node_edge_localizer
{
Localizer::Localizer(void)
: local_nh_("~")
, map_subscribed_(false)
, last_odom_timestamp_(0)
, first_odom_callback_(true)
{
    nh_.advertise<nav_msgs::Odometry>("estimated_pose", 1);
    nh_.subscribe("odom", 1, &Localizer::odom_callback, this, ros::TransportHints().reliable().tcpNoDelay(true));
    nh_.subscribe("node_edge_map/map", 1, &Localizer::map_callback, this, ros::TransportHints().reliable().tcpNoDelay(true));
}

void Localizer::odom_callback(const nav_msgs::OdometryConstPtr& msg)
{
    Pose p = {
        Eigen::Vector3d(msg->pose.pose.position.x,
                        msg->pose.pose.position.y,
                        msg->pose.pose.position.z),
        tf2::getYaw(msg->pose.pose.orientation)
    };
    double odom_timestamp = msg->header.stamp.toSec();
    
    if(first_odom_callback_){
        first_odom_callback_ = false;
        last_odom_timestamp_ = odom_timestamp;
        last_odom_pose_ = p;
        return;
    }

    nav_msgs::Odometry estimated_pose = convert_pose_to_msg(estimated_pose_);
    estimated_pose.header = msg->header;
    estimated_pose.pose.covariance = msg->pose.covariance;
    estimated_pose_pub_.publish(estimated_pose);
    last_odom_timestamp_ = odom_timestamp;
    last_odom_pose_ = p;
}

void Localizer::map_callback(const amsl_navigation_msgs::NodeEdgeMapConstPtr& msg)
{
    amsl_navigation_msgs::NodeEdgeMap map = *msg;
    nemi_.set_map(map);
    map_subscribed_ = true;
}

void Localizer::initialize(void)
{

}

nav_msgs::Odometry Localizer::convert_pose_to_msg(const Pose& p)
{
    nav_msgs::Odometry o;
    o.pose.pose.position.x = p.position_(0);
    o.pose.pose.position.y = p.position_(1);
    o.pose.pose.position.z = p.position_(2);
    tf2::Quaternion q;
    q.setEuler(p.yaw_, 0, 0);
    o.pose.pose.orientation = tf2::toMsg(q);
    return o;
}

void Localizer::process(void)
{
    ros::spin();
}
}// namespace node_edge_localizer