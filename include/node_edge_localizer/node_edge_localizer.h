#ifndef __NODE_EDGE_LOCALIZER_H
#define __NODE_EDGE_LOCALIZER_H

#include <random>

#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

#include "Eigen/Dense"
#include "Eigen/Geometry"

#include "amsl_navigation_msgs/Node.h"
#include "amsl_navigation_msgs/Edge.h"
#include "amsl_navigation_msgs/NodeEdgeMap.h"

#include "node_edge_localizer/node_edge_particle.h"
#include "node_edge_localizer/calculation.h"
#include "node_edge_localizer/node_edge_map_management.h"

class NodeEdgeLocalizer
{
public:
    NodeEdgeLocalizer(void);

    void map_callback(const amsl_navigation_msgs::NodeEdgeMapConstPtr&);
    void odom_callback(const nav_msgs::OdometryConstPtr&);
    void intersection_callback(const std_msgs::BoolConstPtr&);
    void process(void);
    void clustering_trajectories(void);
    void initialize(void);
    void correct(void);
    bool calculate_affine_tranformation(const int, double&, double&, Eigen::Affine3d&);
    void calculate_affine_transformation_tentatively(Eigen::Affine3d&);
    void get_intersection_from_trajectories(std::vector<Eigen::Vector3d>&, std::vector<Eigen::Vector3d>&, Eigen::Vector3d&);
    double calculate_trajectory_curvature(void);
    void publish_pose(void);
    void publish_particles(void);
    void particle_filter(int&, bool&);
    void resampling(void);
    void correct_trajectories(int, const Eigen::Affine3d&);
    void clear(int);
    void visualize_lines(void);
    void remove_curve_from_trajectory(std::vector<Eigen::Vector3d>&);
    void publish_edge(int, bool);
    void publish_odom_tf(Eigen::Vector3d&, double);
    void remove_shorter_line_from_trajectories(const int);
    void set_particle_to_near_edge(bool, int, NodeEdgeParticle&);

private:
    double HZ;
    int INIT_NODE0_ID;
    int INIT_NODE1_ID;
    double INIT_PROGRESS;
    double INIT_YAW;
    double CURVATURE_THRESHOLD;
    int POSE_NUM_PCA;
    int MIN_LINE_SIZE;
    double MIN_LINE_LENGTH;
    bool ENABLE_TF;
    bool USE_ORIENTATION_Z_AS_YAW;
    int PARTICLES_NUM;
    double NOISE_SIGMA;
    double EDGE_DECISION_THRESHOLD;
    double SAME_TRAJECTORY_ANGLE_THRESHOLD;
    double CONTINUOUS_LINE_THRESHOLD;
    bool ENABLE_ODOM_TF;
    double CORRECTION_REJECTION_ANGLE_DIFFERENCE_THRESHOLD;
    int RESAMPLING_INTERVAL;
    double EDGE_CERTAIN_THRESHOLD;

    ros::NodeHandle nh;
    ros::NodeHandle private_nh;

    ros::Publisher edge_pub;
    ros::Publisher odom_pub;
    ros::Publisher particles_pub;
    ros::Publisher lines_pub;
    ros::Publisher edge_marker_pub;
    ros::Subscriber map_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber intersection_sub;

    tf::TransformListener listener;
    tf::TransformBroadcaster broadcaster;

    NodeEdgeMapManagement nemm;
    amsl_navigation_msgs::Edge estimated_edge;
    bool map_subscribed;
    bool odom_updated;
    bool intersection_flag;
    Eigen::Vector3d estimated_pose;
    Eigen::Vector3d init_estimated_pose;
    double estimated_yaw;
    bool init_flag;
    bool clear_flag;
    // estimated edge(line) from odom
    std::vector<Eigen::Vector3d> trajectory;
    // estimated edges from odom
    std::vector<std::vector<Eigen::Vector3d> > linear_trajectories;
    // correct odom to edge
    Eigen::Affine3d odom_correction;
    double yaw_correction;

    bool first_edge_flag;
    std::string robot_frame_id;
    std::string odom_frame_id;

    // for particle filter
    std::vector<NodeEdgeParticle> particles;
    double robot_moved_distance;

    int correction_count = 0;// count up
    int tentative_correction_count;// count down

    ros::Time odom_time;
};

#endif// __NODE_EDGE_LOCALIZER_H
