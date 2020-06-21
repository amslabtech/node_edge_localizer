/**
 * @file localizer.h
 * @author amsl
 */
#ifndef __NODE_EDGE_LOCALIZER_LOCALIZER_H
#define __NODE_EDGE_LOCALIZER_LOCALIZER_H

#include <random>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>

#include <Eigen/Dense>

#include "amsl_navigation_msgs/Node.h"
#include "amsl_navigation_msgs/Edge.h"
#include "amsl_navigation_msgs/NodeEdgeMap.h"
#include "amsl_navigation_managers/node_edge_map_interface.h"

namespace node_edge_localizer
{
struct Pose
{
    // x, y, z
    Eigen::Vector3d position_;
    double yaw_;
};

struct Particle
{
    Pose pose_;
    double likelihood_;
};

class Localizer
{
public:
    Localizer(void);

    void odom_callback(const nav_msgs::OdometryConstPtr& msg);
    void map_callback(const amsl_navigation_msgs::NodeEdgeMapConstPtr& msg);
    void initialize(void);
    nav_msgs::Odometry convert_pose_to_msg(const Pose& p);
    void publish_map_to_odom_tf(const ros::Time& stamp, const std::string& odom_frame_id, const std::string& child_frame_id, const geometry_msgs::Pose& pose);
    void process(void);
protected:
    bool ENABLE_TF_;
    int PARTICLE_NUM_;
    //! initial position x
    double INIT_X_;
    //! initial position y
    double INIT_Y_;
    //! initial orientation yaw
    double INIT_YAW_;
    //! stddev of initial position
    double INIT_SIGMA_XY_;
    //! stddev of initial orientation 
    double INIT_SIGMA_YAW_;
    //! stddev of position transition
    double SIGMA_XY_;
    //! stddev of orientatoin transition
    double SIGMA_YAW_;

    ros::NodeHandle nh_;
    ros::NodeHandle local_nh_;
    ros::Publisher estimated_pose_pub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber map_sub_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfb_;
    Pose estimated_pose_;
    NodeEdgeMapInterface nemi_;
    bool map_subscribed_;
    double last_odom_timestamp_;
    bool first_odom_callback_;
    Pose last_odom_pose_;
    std::vector<Particle> particles_;
    std::random_device rd_;
    std::mt19937 engine_;
};
}// namespace node_edge_localizer

#endif// __NODE_EDGE_LOCALIZER_LOCALZER_H