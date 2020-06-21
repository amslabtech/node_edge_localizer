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

    local_nh_.param<bool>("ENABLE_TF", ENABLE_TF_, true);
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

    // publish estiamted pose
    nav_msgs::Odometry estimated_pose = convert_pose_to_msg(estimated_pose_);
    estimated_pose.header = msg->header;
    estimated_pose.pose.covariance = msg->pose.covariance;
    estimated_pose_pub_.publish(estimated_pose);

    // publish tf from map to odom
    if(ENABLE_TF_){
        publish_map_to_odom_tf(msg->header.stamp, 
                               msg->header.frame_id, 
                               msg->child_frame_id, 
                               estimated_pose.pose.pose);
    }

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

void Localizer::publish_map_to_odom_tf(const ros::Time& stamp, const std::string& odom_frame_id, const std::string& child_frame_id, const geometry_msgs::Pose& pose)
{
    tf2::Transform map_to_robot_tf;
    tf2::convert(pose, map_to_robot_tf);
    geometry_msgs::PoseStamped robot_to_map_pose;
    robot_to_map_pose.header.frame_id = child_frame_id;
    robot_to_map_pose.header.stamp = stamp;
    tf2::toMsg(map_to_robot_tf.inverse(), robot_to_map_pose.pose);
    geometry_msgs::PoseStamped odom_to_map_pose;
    try{
        tf_->transform(robot_to_map_pose, odom_to_map_pose, odom_frame_id);
    }catch(tf2::TransformException ex){
        std::cout << ex.what() << std::endl;
        return;
    }
    tf2::Transform odom_to_map_tf;
    tf2::convert(odom_to_map_pose.pose, odom_to_map_tf);
    geometry_msgs::TransformStamped map_to_odom_tf;
    map_to_odom_tf.header.stamp = stamp;
    map_to_odom_tf.header.frame_id = nemi_.get_map_header_frame_id();
    map_to_odom_tf.child_frame_id = child_frame_id;
    tf2::convert(odom_to_map_tf.inverse(), map_to_odom_tf.transform);
    tfb_->sendTransform(map_to_odom_tf);
}

void Localizer::process(void)
{
    ros::spin();
}
}// namespace node_edge_localizer