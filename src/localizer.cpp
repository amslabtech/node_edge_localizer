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
, engine_(rd_())
{
    particles_pub_ = nh_.advertise<geometry_msgs::PoseArray>("estimated_pose/particles", 1);
    nh_.advertise<nav_msgs::Odometry>("estimated_pose", 1);
    nh_.subscribe("odom", 1, &Localizer::odom_callback, this, ros::TransportHints().reliable().tcpNoDelay(true));
    nh_.subscribe("node_edge_map/map", 1, &Localizer::map_callback, this, ros::TransportHints().reliable().tcpNoDelay(true));

    local_nh_.param<bool>("ENABLE_TF", ENABLE_TF_, true);
    local_nh_.param<int>("PARTICLE_NUM", PARTICLE_NUM_, 1000);
    local_nh_.param<double>("INIT_X", INIT_X_, 0.0);
    local_nh_.param<double>("INIT_Y", INIT_Y_, 0.0);
    local_nh_.param<double>("INIT_YAW", INIT_YAW_, 0.0);
    local_nh_.param<double>("INIT_SIGMA_XY", INIT_SIGMA_XY_, 1.0);
    local_nh_.param<double>("INIT_SIGMA_YAW", INIT_SIGMA_YAW_, M_PI / 3.0);
    local_nh_.param<double>("SIGMA_XY", SIGMA_XY_, 0.1);
    local_nh_.param<double>("SIGMA_YAW", SIGMA_YAW_, 0.1);

    initialize();
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
        first_odom_pose_ = p;
        last_odom_timestamp_ = odom_timestamp;
        return;
    }
    // offset odometry
    p.yaw_ -= first_odom_pose_.yaw_;
    Calculation::pi_2_pi(p.yaw_);
    p.position_ -= first_odom_pose_.position_;
    Eigen::AngleAxis<double> first_odom_yaw_rotation(-first_odom_pose_.yaw_, Eigen::Vector3d::UnitZ());
    p.position_ = first_odom_yaw_rotation * p.position_;

    // get robot motion
    double dt = odom_timestamp - last_odom_timestamp_;
    if(dt == 0.0){
        std::cout << "error: dt must be > 0" << std::endl;
        return;
    }
    Eigen::Vector3d velocity = (p.position_ - last_odom_pose_.position_) / dt;
    Eigen::AngleAxis<double> last_yaw_rotation(-last_odom_pose_.yaw_, Eigen::Vector3d::UnitZ());
    velocity = last_yaw_rotation * velocity; 
    double yawrate = Calculation::pi_2_pi(p.yaw_ - last_odom_pose_.yaw_) / dt;

    move_particles(velocity, yawrate, dt);

    // publish estiamted pose
    nav_msgs::Odometry estimated_pose = convert_pose_to_msg(estimated_pose_);
    estimated_pose.header = msg->header;
    estimated_pose.pose.covariance = msg->pose.covariance;
    estimated_pose_pub_.publish(estimated_pose);

    publish_particles(estimated_pose.header.stamp, estimated_pose.header.frame_id);

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
    first_odom_pose_.position_ = Eigen::Vector3d::Zero();
    first_odom_pose_.yaw_ = 0.0;

    // initialize particles
    particles_.clear();
    std::normal_distribution<> noise_xy(0.0, INIT_SIGMA_XY_);
    std::normal_distribution<> noise_yaw(0.0, INIT_SIGMA_YAW_);
    for(int i=0;i<PARTICLE_NUM_;i++){
        Particle p{
            Pose{
                Eigen::Vector3d(INIT_X_ + noise_xy(engine_), INIT_Y_ + noise_xy(engine_), 0),
                INIT_YAW_ + noise_yaw(engine_)
            },
            1.0 / (double)(PARTICLE_NUM_)
        };
        particles_.push_back(p);
    }
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

void Localizer::move_particles(const Eigen::Vector3d& velocity, const double yawrate, const double dt)
{
    std::normal_distribution<> noise_xy(0.0, SIGMA_XY_);
    std::normal_distribution<> noise_yaw(0.0, SIGMA_YAW_);
    for(auto& particle : particles_){
        double dx = (velocity(0) + noise_xy(engine_)) * dt;
        double dy = (velocity(1) + noise_xy(engine_)) * dt;
        double dyaw = (yawrate + noise_yaw(engine_)) * dt;
        Eigen::Vector3d t(dx, dy, 0.0);
        Eigen::Matrix3d r;
        r = Eigen::AngleAxisd(dyaw + particle.pose_.yaw_, Eigen::Vector3d::UnitZ());
        particle.pose_.position_ = r * t + particle.pose_.position_;
    }
}

void Localizer::publish_particles(const ros::Time& stamp, const std::string& frame_id)
{
    geometry_msgs::PoseArray particles_msg;
    particles_msg.header.stamp = stamp;
    particles_msg.header.frame_id = frame_id;
    const unsigned int size = particles_.size();
    particles_msg.poses.resize(size);
    for(unsigned int i=0;i<size;i++){
        geometry_msgs::Pose p;
        p.position.x = particles_[i].pose_.position_(0);
        p.position.y = particles_[i].pose_.position_(1);
        tf2::Quaternion q;
        q.setEuler(particles_[i].pose_.yaw_, 0, 0);
        p.orientation = tf2::toMsg(q);
        particles_msg.poses[i] = p;
    }
    particles_pub_.publish(particles_msg);
}

void Localizer::process(void)
{
    ros::spin();
}
}// namespace node_edge_localizer