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
    void observation_map_callback(const nav_msgs::OccupancyGridConstPtr& msg);
    void initialize(void);
    void initialize(double x, double y, double yaw);
    void initialize_particles(double x, double y, double yaw);
    void initialize_particles_uniform(double x, double y, double yaw);
    nav_msgs::Odometry convert_pose_to_msg(const Pose& p);
    void publish_map_to_odom_tf(const ros::Time& stamp, const std::string& odom_frame_id, const std::string& robot_frame_id, const geometry_msgs::Pose& pose);
    void publish_odom_to_robot_tf(const ros::Time& stamp, const std::string& odom_frame_id, const std::string& robot_frame_id, const Pose& pose);
    void move_particles(const Eigen::Vector3d& velocity, const double yawrate, const double dt);
    void publish_particles(const ros::Time& stamp, const std::string& frame_id);
    std::tuple<Pose, std::vector<double>> get_estimation_result_from_particles(void);
    std::tuple<Pose, std::vector<double>> get_estimation_result_from_particles_max_weight(void);
    void print_pose(const geometry_msgs::Pose& pose);
    void normalize_particles_weight(void);
    geometry_msgs::Quaternion get_quaternion_msg_from_yaw(const double yaw);
    double compute_num_of_effective_particles(void);
    void resample_particles(void);
    double compute_particle_likelihood_from_motion(const Eigen::Vector3d& dp_r, const double dyaw_r, const Eigen::Vector3d& dp, const double dyaw);
    void publish_distance_map(const DistanceMap& dm, const std::string& frame_id, const ros::Time& stamp);
    void compute_particle_likelihood(const std::vector<Eigen::Vector2d>& free_vectors, const std::vector<Eigen::Vector2d>& obstacle_vectors);
    double compute_average_particle_wight(void);
    void remove_reversed_edges_from_map(amsl_navigation_msgs::NodeEdgeMap& map);
    std::vector<unsigned int> get_near_edge_indices(unsigned int edge_index);
    std::vector<std::vector<unsigned int>> get_connected_edge_indices(void);
    double compute_likelihood(const Pose& pose, const std::vector<Eigen::Vector2d>& free_vectors, const std::vector<Eigen::Vector2d>& obstacle_vectors, const double weight);
    void subsample_observed_points(std::vector<Eigen::Vector2d>& free_vectors, std::vector<Eigen::Vector2d>& obstacle_vectors);
    void process(void);
protected:
    bool enable_tf_;
    bool enable_odom_tf_;
    unsigned int particle_num_;
    /// initial position x
    double init_x_;
    /// initial position y
    double init_y_;
    /// initial orientation yaw
    double init_yaw_;
    /// stddev of initial position
    double init_sigma_xy_;
    /// stddev of initial orientation 
    double init_sigma_yaw_;
    /// stddev of position transition
    double sigma_xy_;
    /// stddev of orientatoin transition
    double sigma_yaw_;
    /// resolution of DistanceMap [m/cell]
    double dm_resolution_;
    /// threshold for reciprocal of sum of squares of particles weight
    double resampling_threshold_;
    /// parameter for tuning likelihood
    double observation_distance_offset_;
    double alpha_fast_;
    double alpha_slow_;
    // \in [0, 1]
    double obstacle_ratio_;
    unsigned int input_point_num_;

    ros::NodeHandle nh_;
    ros::NodeHandle local_nh_;
    ros::Publisher estimated_pose_pub_;
    ros::Publisher particles_pub_;
    ros::Publisher distance_map_pub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber map_sub_;
    ros::Subscriber initial_pose_sub_;
    ros::Subscriber observation_map_sub_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::shared_ptr<tf2_ros::TransformListener> tfl_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfb_;
    Pose estimated_pose_;
    amsl_navigation_msgs::NodeEdgeMap map_;
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
    std::string robot_frame_;
    double w_fast_;
    double w_slow_;
    std::vector<std::vector<unsigned int>> connected_edge_indices_;
};
}// namespace node_edge_localizer

#endif// __NODE_EDGE_LOCALIZER_LOCALZER_H