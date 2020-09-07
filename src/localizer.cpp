/**
 * @file localizer.cpp 
 * @author amsl 
 */

#include "node_edge_localizer/localizer.h"

namespace node_edge_localizer
{
Localizer::Localizer(void)
: local_nh_("~")
, map_received_(false)
, last_odom_timestamp_(0)
, first_odom_callback_(true)
, engine_(rd_())
, robot_frame_("")
, w_fast_(0.0)
, w_slow_(0.0)
{
    particles_pub_ = nh_.advertise<geometry_msgs::PoseArray>("estimated_pose/particles", 1);
    estimated_pose_pub_ = nh_.advertise<nav_msgs::Odometry>("estimated_pose/pose", 1);
    distance_map_pub_ = local_nh_.advertise<nav_msgs::OccupancyGrid>("distance_map", 1);
    odom_sub_ = nh_.subscribe("odom", 1, &Localizer::odom_callback, this, ros::TransportHints().reliable().tcpNoDelay(true));
    map_sub_ = nh_.subscribe("node_edge_map/map", 1, &Localizer::map_callback, this, ros::TransportHints().reliable().tcpNoDelay(true));
    initial_pose_sub_ = nh_.subscribe("initialpose", 1, &Localizer::initial_pose_callback, this, ros::TransportHints().reliable().tcpNoDelay(true));
    observation_map_sub_ = nh_.subscribe("observation_map", 1, &Localizer::observation_map_callback , this, ros::TransportHints().reliable().tcpNoDelay(true));

    tf_ = std::make_shared<tf2_ros::Buffer>();
    tf_->setUsingDedicatedThread(true);
    tfb_ = std::make_shared<tf2_ros::TransformBroadcaster>();
    tfl_ = std::make_shared<tf2_ros::TransformListener>(*tf_);

    local_nh_.param<bool>("enable_tf", enable_tf_, true);
    local_nh_.param<bool>("enable_odom_tf", enable_odom_tf_, false);
    local_nh_.param<int>("particle_num", particle_num_, 100);
    local_nh_.param<double>("init_x", init_x_, 0.0);
    local_nh_.param<double>("init_y", init_y_, 0.0);
    local_nh_.param<double>("init_yaw", init_yaw_, 0.0);
    local_nh_.param<double>("init_sigma_xy", init_sigma_xy_, 0.5);
    local_nh_.param<double>("init_sigma_yaw", init_sigma_yaw_, 0.2);
    local_nh_.param<double>("sigma_xy", sigma_xy_, 0.1);
    local_nh_.param<double>("sigma_yaw", sigma_yaw_, 0.1);
    local_nh_.param<double>("distance_map/resolution", dm_resolution_, 0.1);
    local_nh_.param<double>("resampling_threshold", resampling_threshold_, particle_num_ * 0.5);
    local_nh_.param<double>("observation_distance_offset", observation_distance_offset_, 2.0);
    local_nh_.param<double>("alpha_slow", alpha_slow_, 0.001);
    local_nh_.param<double>("alpha_fast", alpha_fast_, 0.1);

    ROS_INFO_STREAM("enable_tf: " << enable_tf_);
    ROS_INFO_STREAM("enable_odom_tf: " << enable_odom_tf_);
    ROS_INFO_STREAM("particle_num: " << particle_num_);
    ROS_INFO_STREAM("init_x: " << init_x_);
    ROS_INFO_STREAM("init_y: " << init_y_);
    ROS_INFO_STREAM("init_yaw: " << init_yaw_);
    ROS_INFO_STREAM("init_sigma_xy: " << init_sigma_xy_);
    ROS_INFO_STREAM("init_sigma_yaw: " << init_sigma_yaw_);
    ROS_INFO_STREAM("sigma_xy: " << sigma_xy_);
    ROS_INFO_STREAM("sigma_yaw: " << sigma_yaw_);
    ROS_INFO_STREAM("dm_resolution: " << dm_resolution_);
    ROS_INFO_STREAM("resampling_threshold: " << resampling_threshold_);
    ROS_INFO_STREAM("observation_distance_offset: " << observation_distance_offset_);
    ROS_INFO_STREAM("alpha_slow: " << alpha_slow_);
    ROS_INFO_STREAM("alpha_fast: " << alpha_fast_);

    initialize();
}

void Localizer::odom_callback(const nav_msgs::OdometryConstPtr& msg)
{
    if(!map_received_){
        std::cout << ros::this_node::getName() << ": waiting for map..." << std::endl;
        return;
    }
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
        robot_frame_ = msg->child_frame_id;
        if(robot_frame_[0] == '/'){
            robot_frame_.erase(0, 1);
        }
        std::cout << "robot frame is set to " << robot_frame_ << std::endl;
        return;
    }
    // offset odometry
    p.yaw_ -= first_odom_pose_.yaw_;
    Calculation::pi_2_pi(p.yaw_);
    p.position_ -= first_odom_pose_.position_;
    Eigen::AngleAxis<double> first_odom_yaw_rotation(-first_odom_pose_.yaw_, Eigen::Vector3d::UnitZ());
    p.position_ = first_odom_yaw_rotation * p.position_;
    if(enable_odom_tf_){
        publish_odom_to_robot_tf(msg->header.stamp, msg->header.frame_id, msg->child_frame_id, p);
    }

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

    normalize_particles_weight();
    std::vector<double> covariance;
    std::tie(estimated_pose_, covariance) = get_estimation_result_from_particles();

    // publish estiamted pose
    nav_msgs::Odometry estimated_pose = convert_pose_to_msg(estimated_pose_);
    estimated_pose.header.stamp = msg->header.stamp;
    estimated_pose.header.frame_id = nemi_.get_map_header_frame_id();
    estimated_pose.child_frame_id = msg->child_frame_id;
    estimated_pose.twist = msg->twist;
    for(unsigned int i=0;i<36;i++){
        estimated_pose.pose.covariance[i] = covariance[i];
    }
    estimated_pose_pub_.publish(estimated_pose);

    publish_particles(estimated_pose.header.stamp, estimated_pose.header.frame_id);

    // publish tf from map to odom
    if(enable_tf_){
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
    std::cout << ros::this_node::getName() << ": map_callback" << std::endl;
    map_ = *msg;
    std::cout << "remove duplicated edges" << std::endl;
    std::cout << "edge num: " << map_.edges.size() << std::endl;
    remove_reversed_edges_from_map(map_);
    std::cout << "after edge num: " << map_.edges.size() << std::endl;
    for(const auto& e : map_.edges){
        std::cout << e.node0_id << " -> " << e.node1_id << std::endl;
    }
    connected_edge_indices_ = get_connected_edge_indices();
    const unsigned edge_num = map_.edges.size();
    for(unsigned int i=0;i<edge_num;++i){
        std::cout << "edge index: " << i << std::endl;
        std::cout << map_.edges[i].node0_id << " -> " << map_.edges[i].node1_id << std::endl;
        const unsigned int connected_edges_num = connected_edge_indices_[i].size();
        for(unsigned int j=0;j<connected_edges_num;++j){
            std::cout << "\t" << map_.edges[connected_edge_indices_[i][j]].node0_id << " -> " << map_.edges[connected_edge_indices_[i][j]].node1_id << std::endl;
        }
    }
    nemi_.set_map(map_);
    auto start = std::chrono::system_clock::now();
    dm_.make_distance_map(map_, dm_resolution_);
    auto end = std::chrono::system_clock::now();
    map_received_ = true;
    std::cout << "distance map was computed in: " << std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count() << "[ms]" << std::endl;
    publish_distance_map(dm_, msg->header.frame_id, msg->header.stamp);
}

void Localizer::observation_map_callback(const nav_msgs::OccupancyGridConstPtr& msg)
{
    std::cout << "observation_map_callback" << std::endl;
    auto start = std::chrono::system_clock::now();
    if(!map_received_){
        std::cout << ros::this_node::getName() << ": map is not received" << std::endl;
        return;
    }
    if(robot_frame_.empty()){
        std::cout << ros::this_node::getName() << ": robot frame is not set" << std::endl;
        return;
    }
    if(msg->header.frame_id != robot_frame_){
        std::cout << ros::this_node::getName() << ": observation map must be in the robot frame: " << robot_frame_ << std::endl;
        return;
    }
    // get obstacle and free positions from OGM
    const unsigned int size = msg->data.size();
    std::vector<Eigen::Vector2d> free_vectors;
    free_vectors.reserve(size);
    std::vector<Eigen::Vector2d> obstacle_vectors;
    obstacle_vectors.reserve(size);
    for(unsigned int i=0;i<size;i++){
        if(msg->data[i] < 0){
            continue;
        }
        const double x = (i % msg->info.width) * msg->info.resolution + msg->info.origin.position.x;
        const double y = floor(i / msg->info.width) * msg->info.resolution + msg->info.origin.position.y;
        const Eigen::Vector2d v(x, y);
        if(0 <= msg->data[i] && msg->data[i] < 20){
            free_vectors.emplace_back(v); 
        }else if(80 <= msg->data[i]){
            obstacle_vectors.emplace_back(v);
        }
    }
    std::cout << "observed free points: " << free_vectors.size() << std::endl;
    std::cout << "observed obstacle points: " << obstacle_vectors.size() << std::endl;
    compute_particle_likelihood(free_vectors, obstacle_vectors);
    resample_particles();
    auto end = std::chrono::system_clock::now();
    std::cout << "observation_map_callback time: " << std::chrono::duration_cast<std::chrono::microseconds>(end-start).count() << "[us]" << std::endl;
}

void Localizer::initial_pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
    if(msg->header.frame_id != nemi_.get_map_header_frame_id()){
        std::cout << "Initial pose must be in the global frame: " << nemi_.get_map_header_frame_id() << std::endl;
        return;
    }
    std::cout << "received initial pose: " ;
    print_pose(msg->pose.pose);
    initialize(msg->pose.pose.position.x, msg->pose.pose.position.y, tf2::getYaw(msg->pose.pose.orientation));
}

void Localizer::initialize(void)
{
    initialize(init_x_, init_y_, init_yaw_);
}

void Localizer::initialize(double x, double y, double yaw)
{
    initialize_particles(x, y, yaw);
    std::vector<double> covariance;
    std::tie(estimated_pose_, covariance) = get_estimation_result_from_particles();
    std::cout << "initialized pose: " << estimated_pose_.position_.transpose() << ", " << estimated_pose_.yaw_ << std::endl;
    first_odom_callback_ = true;
    first_odom_pose_.position_ = Eigen::Vector3d::Zero();
    first_odom_pose_.yaw_ = 0.0;
    last_odom_timestamp_ = 0.0;
    last_odom_pose_ = Pose();
}

void Localizer::initialize_particles(double x, double y, double yaw)
{
    particles_.clear();
    std::normal_distribution<> noise_xy(0.0, init_sigma_xy_);
    std::normal_distribution<> noise_yaw(0.0, init_sigma_yaw_);
    for(int i=0;i<particle_num_;i++){
        Particle p{
            Pose{
                Eigen::Vector3d(x + noise_xy(engine_), y + noise_xy(engine_), 0),
                yaw + noise_yaw(engine_)
            },
            1.0 / (double)(particle_num_)
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
    q.setRPY(0, 0, p.yaw_);
    o.pose.pose.orientation = tf2::toMsg(q);
    return o;
}

void Localizer::publish_map_to_odom_tf(const ros::Time& stamp, const std::string& odom_frame_id, const std::string& robot_frame_id, const geometry_msgs::Pose& pose)
{
    tf2::Transform map_to_robot_tf;
    tf2::convert(pose, map_to_robot_tf);
    geometry_msgs::PoseStamped robot_to_map_pose;
    robot_to_map_pose.header.frame_id = robot_frame_id;
    robot_to_map_pose.header.stamp = stamp;
    tf2::toMsg(map_to_robot_tf.inverse(), robot_to_map_pose.pose);
    geometry_msgs::PoseStamped odom_to_map_pose;
    try{
        tf_->transform(robot_to_map_pose, odom_to_map_pose, odom_frame_id);
    }catch(tf2::TransformException& ex){
        std::cout << ex.what() << std::endl;
        return;
    }
    tf2::Transform odom_to_map_tf;
    tf2::convert(odom_to_map_pose.pose, odom_to_map_tf);
    geometry_msgs::TransformStamped map_to_odom_tf;
    map_to_odom_tf.header.stamp = stamp;
    map_to_odom_tf.header.frame_id = nemi_.get_map_header_frame_id();
    map_to_odom_tf.child_frame_id = odom_frame_id;
    tf2::convert(odom_to_map_tf.inverse(), map_to_odom_tf.transform);
    tfb_->sendTransform(map_to_odom_tf);
}

void Localizer::publish_odom_to_robot_tf(const ros::Time& stamp, const std::string& odom_frame_id, const std::string& robot_frame_id, const Pose& pose)
{
    geometry_msgs::TransformStamped odom_to_robot_tf;
    odom_to_robot_tf.header.stamp = stamp;
    odom_to_robot_tf.header.frame_id = odom_frame_id;
    odom_to_robot_tf.child_frame_id = robot_frame_id;
    odom_to_robot_tf.transform.translation.x = pose.position_(0);
    odom_to_robot_tf.transform.translation.y = pose.position_(1);
    odom_to_robot_tf.transform.translation.z = pose.position_(2);
    odom_to_robot_tf.transform.rotation = get_quaternion_msg_from_yaw(pose.yaw_); 
    tfb_->sendTransform(odom_to_robot_tf);
}

void Localizer::move_particles(const Eigen::Vector3d& velocity, const double yawrate, const double dt)
{
    const Eigen::Vector3d dp_r = velocity * dt;
    const double dyaw_r = yawrate * dt;
    std::normal_distribution<> noise_xy(0.0, sigma_xy_);
    std::normal_distribution<> noise_yaw(0.0, sigma_yaw_);
    for(auto& particle : particles_){
        double dx = (velocity(0) + noise_xy(engine_)) * dt;
        double dy = (velocity(1) + noise_xy(engine_)) * dt;
        double dyaw = (yawrate + noise_yaw(engine_)) * dt;
        Eigen::Vector3d t(dx, dy, 0.0);
        Eigen::Matrix3d r;
        r = Eigen::AngleAxisd(particle.pose_.yaw_, Eigen::Vector3d::UnitZ());
        particle.pose_.position_ = r * t + particle.pose_.position_;
        particle.pose_.yaw_ = Calculation::pi_2_pi(particle.pose_.yaw_ + dyaw);
        particle.weight_ *= compute_particle_likelihood_from_motion(dp_r, dyaw_r, t, dyaw);
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
        q.setRPY(0, 0, particles_[i].pose_.yaw_);
        p.orientation = tf2::toMsg(q);
        particles_msg.poses[i] = p;
    }
    particles_pub_.publish(particles_msg);
}

std::tuple<Pose, std::vector<double>> Localizer::get_estimation_result_from_particles(void)
{
    Pose p{
        Eigen::Vector3d::Zero(),
        0.0
    };
    std::vector<double> cov(36);
    double direction_x = 0;
    double direction_y = 0;
    const unsigned int num = particles_.size();
    // calculate mean
    for(const auto& particle : particles_){
        p.position_ += particle.weight_ * particle.pose_.position_;
        direction_x += particle.weight_ * cos(particle.pose_.yaw_);
        direction_y += particle.weight_ * sin(particle.pose_.yaw_);
    }
    p.yaw_ = atan2(direction_y, direction_x);
    // calculate covariance
    for(const auto& particle : particles_){
        cov[0 * 6 + 0] += Calculation::square(p.position_(0) - particle.pose_.position_(0));
        cov[0 * 6 + 1] += (p.position_(0) - particle.pose_.position_(0)) * (p.position_(1) - particle.pose_.position_(1));
        cov[1 * 6 + 1] += Calculation::square(p.position_(1) - particle.pose_.position_(1));
    }
    cov[1 * 6 + 0] = cov[0 * 6 + 1];
    // TODO: angle covariance
    for(auto& c : cov){
        c /= (double)num;
    }
    return std::forward_as_tuple(p, cov);
}

void Localizer::print_pose(const geometry_msgs::Pose& pose)
{
    std::cout << "(" << pose.position.x << ", " 
              << pose.position.y << ", " 
              << tf2::getYaw(pose.orientation) << ")" << std::endl;
}

void Localizer::normalize_particles_weight(void)
{
    double sum = 0;
    for(const auto& p : particles_){
        sum += p.weight_;
    }
    for(auto& p : particles_){
        p.weight_ /= sum;
    }
}

geometry_msgs::Quaternion Localizer::get_quaternion_msg_from_yaw(const double yaw)
{
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    return tf2::toMsg(q);
}

double Localizer::compute_num_of_effective_particles(void)
{
    double n_e = 0.0;
    for(const auto& p : particles_){
        n_e += p.weight_ * p.weight_;
    }
    n_e = 1.0 / n_e;
    return n_e;
}

void Localizer::resample_particles(void)
{
    std::cout << "preprocess of resampling" << std::endl;
    // compute average before normalization
    const double w_avg = compute_average_particle_wight();
    // std::cout << "w_avg: " << w_avg << std::endl;;
    if(w_slow_ > 0.0){
        w_slow_ += alpha_slow_ * (w_avg - w_slow_);
    }else{
        w_slow_ = w_avg;
    }
    if(w_fast_ > 0.0){
        w_fast_ += alpha_fast_ * (w_avg - w_fast_);
    }else{
        w_fast_ = w_avg;
    }
    // std::cout << "w_fast: " << w_fast_ << std::endl;
    // std::cout << "w_slow: " << w_slow_ << std::endl;;
    const double w_diff = std::max(1.0 - w_fast_ / w_slow_, 0.0);
    // std::cout << "w_diff: " << w_diff << std::endl;;
    std::cout << w_diff * 100 << " percent of particles will be randomly placed" << std::endl;

    normalize_particles_weight();
    const double effective_num_of_particles = compute_num_of_effective_particles();
    std::cout << "n_e: " << effective_num_of_particles << std::endl;
    if(!(effective_num_of_particles < resampling_threshold_)){
        return;
    }

    std::cout << "resampling" << std::endl;
    const unsigned int n = particles_.size();

    std::uniform_real_distribution<> dist(0.0, 1.0);
    std::vector<Particle> new_particles(n);

    // noise for random sampling
    std::normal_distribution<> noise_xy(0.0, init_sigma_xy_);
    std::normal_distribution<> noise_yaw(0.0, init_sigma_yaw_);

    // cumulative sum of particle weight
    std::vector<double> c(n + 1);
    c[0] = 0.0;
    for(unsigned int i=0;i<n;i++){
        c[i+1] = c[i] + particles_[i].weight_;
    }
    std::uniform_real_distribution<> dist_for_sample(0.0, c.back());

    for(unsigned int i=0;i<n;i++){
        if(dist(engine_) > w_diff){
            // sample from existing particles
            double r = dist_for_sample(engine_);
            for(unsigned int j=0;j<n;j++){
                if((c[j] <= r) && (r < c[j+1])){
                    new_particles[i] = particles_[j];
                    new_particles[i].weight_ = 1.0;
                break;
            }
        }
        }else{
            // random sampling
            Particle p{
                Pose{
                    Eigen::Vector3d(estimated_pose_.position_(0) + noise_xy(engine_), estimated_pose_.position_(1) + noise_xy(engine_), 0),
                    estimated_pose_.yaw_ + noise_yaw(engine_)
                },
                1.0
            };
            new_particles[i] = p;
        }
    }
    particles_ = new_particles;
    if(w_diff > 0.0){
        w_slow_ = 0.0;
        w_fast_ = 0.0;
    }
}

double Localizer::compute_particle_likelihood_from_motion(const Eigen::Vector3d& dp_r, const double dyaw_r, const Eigen::Vector3d& dp, const double dyaw)
{
    return exp(-(dp_r - dp).norm()) + exp(-abs(dyaw_r - dyaw));
}

void Localizer::publish_distance_map(const DistanceMap& dm, const std::string& frame_id, const ros::Time& stamp)
{
    std::vector<EdgeIndexWithDistance> data;
    double min_x, max_x, min_y, max_y;
    std::tie(data, min_x, max_x, min_y, max_y) = dm.get_data();
    std::cout << "DistanceMap data:" << std::endl;
    std::cout << "\tmin_x: " << min_x << std::endl;
    std::cout << "\tmax_x: " << max_x << std::endl;
    std::cout << "\tmin_y: " << min_y << std::endl;
    std::cout << "\tmax_y: " << max_y << std::endl;
    const unsigned int width = (max_x - min_x) / dm_resolution_;
    const unsigned int height = (max_y - min_y) / dm_resolution_;
    std::cout << "\twidth: " << width << std::endl;
    std::cout << "\theight: " << height << std::endl;
    nav_msgs::OccupancyGrid og;
    og.header.frame_id = frame_id;
    og.header.stamp = stamp;
    og.info.resolution = dm_resolution_;
    og.info.width = width;
    og.info.height = height;
    og.info.origin.position.x = min_x;
    og.info.origin.position.y = min_y;
    og.info.origin.orientation = get_quaternion_msg_from_yaw(0);
    const unsigned int size = data.size();
    og.data.resize(size);
    for(unsigned int i=0;i<size;i++){
        og.data[i] = std::min(data[i].distance_, 20.0) / 20.0 * 100;
    }
    distance_map_pub_.publish(og);
}

void Localizer::compute_particle_likelihood(const std::vector<Eigen::Vector2d>& free_vectors, const std::vector<Eigen::Vector2d>& obstacle_vectors)
{
    for(auto& p : particles_){
        const unsigned int p_edge_index = dm_.get_nearest_edge_index(p.pose_.position_(0), p.pose_.position_(1));
        // std::cout << "p at " << p.pose_.position_.segment(0, 2).transpose() << ", " << p.pose_.yaw_ << ", " << p_edge_index << std::endl;
        // transform observed points to each particle frame
        const Eigen::Translation2d trans(p.pose_.position_(0), p.pose_.position_(1));
        Eigen::Matrix2d rot;
        rot << cos(p.pose_.yaw_), -sin(p.pose_.yaw_),
               sin(p.pose_.yaw_),  cos(p.pose_.yaw_);
        const Eigen::Affine2d affine = rot * trans;
        double f_w = 0.0;
        double o_w = 0.0;
        int c = 0;
        for(const auto& f : free_vectors){
            const Eigen::Vector2d v = affine * f;
            // TODO: to be updated
            // if free(road) area is near edges, the likelihood should be higher
            // f_w += 1.0 - std::min(1.0, dm_.get_min_distance_from_edge(v(0), v(1)) / observation_distance_offset_); 
            const double d = 1 - std::min(1.0, dm_.get_min_distance_from_edge(v(0), v(1)) / observation_distance_offset_);
            f_w += d;
        }
        p.weight_ *= f_w;
        // std::cout << "f_w: " << f_w << std::endl;
        for(const auto& o : obstacle_vectors){
            const Eigen::Vector2d v = affine * o;
            // TODO: to be updated
            // if obstacle(wall, grass,...) area is near edges, the likelihood should be lower 
            const double d = std::min(1.0, dm_.get_min_distance_from_edge(v(0), v(1)) / observation_distance_offset_);
            const unsigned int o_index = dm_.get_nearest_edge_index(v(0), v(1));
            if(o_index == p_edge_index){
                o_w += d; 
            }
        }
        p.weight_ *= o_w;
        if(p.weight_ < 1e-6){
            p.weight_ = 1e-6;
        }
        // std::cout << "o_w: " << o_w << std::endl;
        // std::cout << "w: " << p.weight_ << std::endl;
        // std::cout << p.pose_.position_(0) << ", " << p.pose_.position_(1) << ", " << p.pose_.yaw_ << " >> " << p.weight_ << std::endl;
    }
}

double Localizer::compute_average_particle_wight(void)
{
    double w_avg = 0;
    for(const auto& p : particles_){
        w_avg += p.weight_;
    }
    w_avg /= particles_.size();
    return w_avg;
}

void Localizer::remove_reversed_edges_from_map(amsl_navigation_msgs::NodeEdgeMap& map)
{
    for(auto it=map.edges.begin();it!=map.edges.end();){
        for(auto it2=it;it2!=map.edges.end();){
            if(!((it->node0_id == it2->node1_id) && (it->node1_id == it2->node0_id))){
                ++it2;
            }else{
                it2 = map.edges.erase(it2);
                break;
            }
        }
        ++it;
    }
}

std::vector<unsigned int> Localizer::get_near_edge_indices(unsigned int edge_index)
{
    std::vector<unsigned int> near_edge_indices;
    // get node 0 and 1
    const unsigned int n0 = map_.edges[edge_index].node0_id;
    const unsigned int n1 = map_.edges[edge_index].node1_id;
    // get edges connected to node 0 and 1
    const unsigned int edge_num = map_.edges.size();
    for(unsigned int i=0;i<edge_num;++i){
        if(edge_index != i){
            if((n0 == map_.edges[i].node0_id) ||
               (n0 == map_.edges[i].node1_id) ||
               (n1 == map_.edges[i].node0_id) ||
               (n1 == map_.edges[i].node1_id)){
                near_edge_indices.emplace_back(i);
            }
        }
    }
    return near_edge_indices;
}

std::vector<std::vector<unsigned int>> Localizer::get_connected_edge_indices(void)
{
    std::vector<std::vector<unsigned int>> connected_edge_indices;
    const unsigned edge_num = map_.edges.size();
    for(unsigned int i=0;i<edge_num;++i){
        connected_edge_indices.emplace_back(get_near_edge_indices(i));
    }
    return connected_edge_indices;
}

void Localizer::process(void)
{
    ros::spin();
}
}// namespace node_edge_localizer