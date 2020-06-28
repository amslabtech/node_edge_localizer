/**
 * @file localizer.h
 * @author amsl
 */
#ifndef __NODE_EDGE_LOCALIZER_LOCALIZER_H
#define __NODE_EDGE_LOCALIZER_LOCALIZER_H

#include <chrono>
#include <random>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseArray.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <Eigen/Dense>

#include "amsl_navigation_msgs/Node.h"
#include "amsl_navigation_msgs/Edge.h"
#include "amsl_navigation_msgs/NodeEdgeMap.h"
#include "amsl_navigation_managers/node_edge_map_interface.h"
#include "node_edge_localizer/calculation.h"
#include "node_edge_localizer/distance_map.h"

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
    double weight_;
};

class Localizer
{
public:
    Localizer(void);

    void odom_callback(const nav_msgs::OdometryConstPtr& msg);
    void map_callback(const amsl_navigation_msgs::NodeEdgeMapConstPtr& msg);
    void initial_pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
    void observation_map_callback(const nav_msgs::OccupancyGrid& msg);
    void initialize(void);
    void initialize(double x, double y, double yaw);
    void initialize_particles(double x, double y, double yaw);
    nav_msgs::Odometry convert_pose_to_msg(const Pose& p);
    void publish_map_to_odom_tf(const ros::Time& stamp, const std::string& odom_frame_id, const std::string& robot_frame_id, const geometry_msgs::Pose& pose);
    void publish_odom_to_robot_tf(const ros::Time& stamp, const std::string& odom_frame_id, const std::string& robot_frame_id, const Pose& pose);
    void move_particles(const Eigen::Vector3d& velocity, const double yawrate, const double dt);
    void publish_particles(const ros::Time& stamp, const std::string& frame_id);
    std::tuple<Pose, std::vector<double>> get_estimation_result_from_particles(void);
    void print_pose(const geometry_msgs::Pose& pose);
    void normalize_particles_weight(void);
    geometry_msgs::Quaternion get_quaternion_msg_from_yaw(const double yaw);
    double compute_num_of_effective_particles(void);
    void resample_particles(void);
    double compute_particle_likelihood_from_motion(const Eigen::Vector3d& dp_r, const double dyaw_r, const Eigen::Vector3d& dp, const double dyaw);
    void process(void);
protected:
    bool ENABLE_TF_;
    bool ENABLE_ODOM_TF_;
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
    //! resolution of DistanceMap [m/cell]
    double DM_RESOLUTION_;
    //! threshold for reciprocal of sum of squares of particles weight
    double RESAMPLING_THRESHOLD_;

    ros::NodeHandle nh_;
    ros::NodeHandle local_nh_;
    ros::Publisher estimated_pose_pub_;
    ros::Publisher particles_pub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber map_sub_;
    ros::Subscriber initial_pose_sub_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::shared_ptr<tf2_ros::TransformListener> tfl_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfb_;
    Pose estimated_pose_;
    NodeEdgeMapInterface nemi_;
    bool map_received_;
    double last_odom_timestamp_;
    bool first_odom_callback_;
    Pose last_odom_pose_;
    std::vector<Particle> particles_;
    std::random_device rd_;
    std::mt19937 engine_;
    Pose first_odom_pose_;
    DistanceMap dm_;
};
}// namespace node_edge_localizer

#endif// __NODE_EDGE_LOCALIZER_LOCALZER_H