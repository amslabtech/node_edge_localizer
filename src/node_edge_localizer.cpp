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
	void set_dead_end_particle_to_edge_near_robot(bool, int, NodeEdgeParticle&);

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

int main(int argc, char** argv)
{
	ros::init(argc, argv, "node_edge_localizer");
	NodeEdgeLocalizer node_edge_localizer;
	node_edge_localizer.process();
	return 0;
}

NodeEdgeLocalizer::NodeEdgeLocalizer(void)
	: private_nh("~")
{
	map_sub = nh.subscribe("/node_edge_map/map", 1, &NodeEdgeLocalizer::map_callback, this);
	odom_sub = nh.subscribe("/odom/complement", 1 ,&NodeEdgeLocalizer::odom_callback, this);
	intersection_sub = nh.subscribe("/intersection_flag", 1 ,&NodeEdgeLocalizer::intersection_callback, this);
	edge_pub = nh.advertise<amsl_navigation_msgs::Edge>("/estimated_pose/edge", 1);
	odom_pub = nh.advertise<nav_msgs::Odometry>("/estimated_pose/pose", 1);
	particles_pub = nh.advertise<geometry_msgs::PoseArray>("/estimated_pose/particles", 1);
	lines_pub = nh.advertise<visualization_msgs::Marker>("/passed_lines/viz", 1);
	edge_marker_pub = nh.advertise<visualization_msgs::Marker>("/estimated_pose/edge/viz", 1);

	private_nh.param("HZ", HZ, {20});
	private_nh.param("INIT_NODE0_ID", INIT_NODE0_ID, {0});
	private_nh.param("INIT_NODE1_ID", INIT_NODE1_ID, {1});
	private_nh.param("INIT_PROGRESS", INIT_PROGRESS, {0.0});
	private_nh.param("INIT_YAW", INIT_YAW, {0.0});
	private_nh.param("CURVATURE_THRESHOLD", CURVATURE_THRESHOLD, {0.01});
	private_nh.param("POSE_NUM_PCA", POSE_NUM_PCA, {50});
	private_nh.param("MIN_LINE_SIZE", MIN_LINE_SIZE, {50});
	private_nh.param("MIN_LINE_LENGTH", MIN_LINE_LENGTH, {3.0});
	private_nh.param("ENABLE_TF", ENABLE_TF, {false});
	private_nh.param("USE_ORIENTATION_Z_AS_YAW", USE_ORIENTATION_Z_AS_YAW, {false});
	private_nh.param("PARTICLES_NUM", PARTICLES_NUM, {1000});
	private_nh.param("NOISE_SIGMA", NOISE_SIGMA, {0.10});
	private_nh.param("EDGE_DECISION_THRESHOLD", EDGE_DECISION_THRESHOLD, {0.5});
	private_nh.param("SAME_TRAJECTORY_ANGLE_THRESHOLD", SAME_TRAJECTORY_ANGLE_THRESHOLD, {M_PI/6.0});
	private_nh.param("CONTINUOUS_LINE_THRESHOLD", CONTINUOUS_LINE_THRESHOLD, {M_PI/7.0});
	private_nh.param("ENABLE_ODOM_TF", ENABLE_ODOM_TF, {false});
	private_nh.param("CORRECTION_REJECTION_ANGLE_DIFFERENCE_THRESHOLD", CORRECTION_REJECTION_ANGLE_DIFFERENCE_THRESHOLD, {M_PI/6.0});
	private_nh.param("RESAMPLING_INTERVAL", RESAMPLING_INTERVAL, {5});
	private_nh.param("EDGE_CERTAIN_THRESHOLD", EDGE_CERTAIN_THRESHOLD, {0.9});

	map_subscribed = false;
	odom_updated = false;
	init_flag = true;
	clear_flag = false;
	first_edge_flag = true;
	intersection_flag = false;
	robot_frame_id = "base_link";
	odom_frame_id = "odom";
	particles.resize(PARTICLES_NUM);
	odom_correction = Eigen::Affine3d::Identity();
	robot_moved_distance = 0.0;
	tentative_correction_count = POSE_NUM_PCA;
	yaw_correction = 0.0;
	estimated_pose = Eigen::Vector3d::Zero();

	std::cout << "=== node_edge_localizer ===" << std::endl;
	std::cout << "HZ: " << HZ << std::endl;
	std::cout << "INIT_NODE0_ID: " << INIT_NODE0_ID << std::endl;
	std::cout << "INIT_NODE1_ID: " << INIT_NODE1_ID << std::endl;
	std::cout << "INIT_PROGRESS: " << INIT_PROGRESS << std::endl;
	std::cout << "INIT_YAW: " << INIT_YAW << std::endl;
	std::cout << "CURVATURE_THRESHOLD: " << CURVATURE_THRESHOLD << std::endl;
	std::cout << "POSE_NUM_PCA: " << POSE_NUM_PCA << std::endl;
	std::cout << "MIN_LINE_SIZE: " << MIN_LINE_SIZE << std::endl;
	std::cout << "MIN_LINE_LENGTH: " << MIN_LINE_LENGTH << std::endl;
	std::cout << "ENABLE_TF: " << ENABLE_TF << std::endl;
	std::cout << "USE_ORIENTATION_Z_AS_YAW: " << USE_ORIENTATION_Z_AS_YAW << std::endl;
	std::cout << "PARTICLES_NUM: " << PARTICLES_NUM << std::endl;
	std::cout << "NOISE_SIGMA: " << NOISE_SIGMA << std::endl;
	std::cout << "EDGE_DECISION_THRESHOLD: " << EDGE_DECISION_THRESHOLD << std::endl;
	std::cout << "SAME_TRAJECTORY_ANGLE_THRESHOLD: " << SAME_TRAJECTORY_ANGLE_THRESHOLD << std::endl;
	std::cout << "CONTINUOUS_LINE_THRESHOLD: " << CONTINUOUS_LINE_THRESHOLD << std::endl;
	std::cout << "ENABLE_ODOM_TF: " << ENABLE_ODOM_TF << std::endl;
	std::cout << "CORRECTION_REJECTION_ANGLE_DIFFERENCE_THRESHOLD: " << CORRECTION_REJECTION_ANGLE_DIFFERENCE_THRESHOLD << std::endl;
	std::cout << "RESAMPLING_INTERVAL: " << RESAMPLING_INTERVAL << std::endl;
	std::cout << "EDGE_CERTAIN_THRESHOLD: " << EDGE_CERTAIN_THRESHOLD << std::endl;
}

void NodeEdgeLocalizer::map_callback(const amsl_navigation_msgs::NodeEdgeMapConstPtr& msg)
{
	amsl_navigation_msgs::NodeEdgeMap map = *msg;
	nemm.set_map(map);
	map_subscribed = true;
}

void NodeEdgeLocalizer::odom_callback(const nav_msgs::OdometryConstPtr& msg)
{
	std::cout << "--- odom calback ---" << std::endl;
	if(!init_flag){
		double start_time = ros::Time::now().toSec();
		static Eigen::Vector3d last_estimated_pose;
		static bool first_odom_callback_flag = true;
		static Eigen::Vector3d first_odom_pose;
		static double first_odom_yaw;

		last_estimated_pose = estimated_pose;

		double odom_yaw;
		if(!USE_ORIENTATION_Z_AS_YAW){
			odom_yaw = tf::getYaw(msg->pose.pose.orientation) + yaw_correction;
		}else{
			odom_yaw = msg->pose.pose.orientation.z + yaw_correction;
		}
		if(first_odom_callback_flag){
			first_odom_yaw = odom_yaw;
			std::cout << "first odom yaw: " << first_odom_yaw << std::endl;
		}
		odom_yaw -= first_odom_yaw;
		odom_yaw = Calculation::pi_2_pi(odom_yaw);
		estimated_yaw = odom_yaw + INIT_YAW;
		estimated_yaw = Calculation::pi_2_pi(estimated_yaw);

		Eigen::Vector3d odom_pose;
		odom_pose << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
		if(first_odom_callback_flag){
			first_odom_pose = odom_pose;
			std::cout << "first odom pose: \n" << first_odom_pose << std::endl;
		}
		odom_pose -= first_odom_pose;
		Eigen::AngleAxis<double> first_odom_yaw_rotation(-first_odom_yaw, Eigen::Vector3d::UnitZ());
		odom_pose = first_odom_yaw_rotation * odom_pose;
		Eigen::AngleAxis<double> init_yaw_rotation(INIT_YAW, Eigen::Vector3d::UnitZ());
		estimated_pose = odom_correction * (init_yaw_rotation * odom_pose + init_estimated_pose);

		std::cout << "odom_pose: \n" << odom_pose << std::endl;
		std::cout << "odom_yaw: \n" << odom_yaw << std::endl;
		std::cout << "odom_correction: \n" << odom_correction.matrix() << std::endl;
		std::cout << "yaw_correction: " << yaw_correction << "[rad]" << std::endl;
		std::cout << "estimated_pose: \n" << estimated_pose << std::endl;
		std::cout << "estimated_yaw: " << estimated_yaw << "[rad]" << std::endl;
		odom_time = msg->header.stamp;
		if(first_odom_callback_flag){
			if(msg->child_frame_id != ""){
				robot_frame_id = msg->child_frame_id;
			}
			if(msg->header.frame_id != ""){
				odom_frame_id = msg->header.frame_id;
			}
			first_odom_callback_flag = false;
		}else{
			odom_updated = true;
			// robot moveed distance for particle
			Eigen::Vector3d move_vector = estimated_pose - last_estimated_pose;
			robot_moved_distance = move_vector.norm();
			double moved_direction = atan2(move_vector(1), move_vector(0));
			double diff_yaw_and_moved_direction = estimated_yaw - moved_direction;
			diff_yaw_and_moved_direction = Calculation::pi_2_pi(diff_yaw_and_moved_direction);
			robot_moved_distance *= cos(diff_yaw_and_moved_direction);
			if(fabs(diff_yaw_and_moved_direction) < M_PI / 2.0){
				// forward
				robot_moved_distance = robot_moved_distance;
			}else{
				// back
				robot_moved_distance = -robot_moved_distance;
			}
		}
		if(ENABLE_ODOM_TF){
			publish_odom_tf(odom_pose, odom_yaw);
		}
		std::cout << "odom callback time: " << ros::Time::now().toSec() - start_time << "[s]" << std::endl;
	}else{
		std::cout << "not initialized !!!" << std::endl;
	}
}

void NodeEdgeLocalizer::intersection_callback(const std_msgs::BoolConstPtr& msg)
{
	intersection_flag = msg->data;
}

void NodeEdgeLocalizer::process(void)
{
	ros::Rate loop_rate(HZ);

	while(ros::ok()){
		if(map_subscribed){
			if(init_flag){
				std::cout << "--- initialize ---" << std::endl;
				initialize();
				std::cout << "waiting for odom..." << std::endl;
			}else if(odom_updated){
				double start_time = ros::Time::now().toSec();
				std::cout << "=== node_edge_localizer ===" << std::endl;
				int unique_edge_index;
				bool unique_edge_flag;
				std::cout << "--- particle filter ---" << std::endl;
				particle_filter(unique_edge_index, unique_edge_flag);
				std::cout << "--- clustering trajectories ---" << std::endl;
				if(!unique_edge_flag){
					tentative_correction_count = POSE_NUM_PCA;
				}
				clustering_trajectories();
				if(tentative_correction_count > 0){
					tentative_correction_count--;
				}
				std::cout << "tentative correction count: " << tentative_correction_count << std::endl;
				std::cout << "--- manage passed edge ---" << std::endl;
				if(unique_edge_flag){
					nemm.manage_passed_edge(unique_edge_index);
				}
				nemm.show_line_edge_ids();
				std::cout << "--- calculate correction ---" << std::endl;
				correct();
				if(clear_flag){
					//clear(unique_edge_index);
				}
				std::cout << "--- publish ---" << std::endl;
				publish_pose();
				publish_particles();
				publish_edge(unique_edge_index, unique_edge_flag);
				visualize_lines();
				odom_updated = false;
				std::cout << "process time: " << ros::Time::now().toSec() - start_time << "[s]" << std::endl;
				std::cout << "===========================" << std::endl;
			}
		}else{
			std::cout << "waiting for map..." << std::endl;
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
}

void NodeEdgeLocalizer::clustering_trajectories(void)
{
	double trajectory_curvature = calculate_trajectory_curvature();
	std::cout << "trajectory curvature: " << trajectory_curvature << std::endl;
	double trajectory_length = Calculation::get_length_of_trajectory(trajectory);
	std::cout << "trajectory length: " << trajectory_length << std::endl;
	if(linear_trajectories.size() > 0){
		std::cout << "linear_trajectory length: " << Calculation::get_length_of_trajectory(linear_trajectories.back()) << std::endl;
	}
	if(trajectory_curvature > CURVATURE_THRESHOLD || trajectory_length > MIN_LINE_LENGTH){
		if((int)trajectory.size() > MIN_LINE_SIZE){
			static Eigen::Vector2d last_slope;
			//static double last_yaw = 0.0;
			if(!first_edge_flag){
				Eigen::Vector2d slope;
				// get slope of current trajectory
				std::cout << "trajectory angle: " << Calculation::get_angle_from_trajectory(trajectory) << std::endl;
				Calculation::get_slope_from_trajectory(trajectory, slope);
				double diff_angle = acos(slope.dot(last_slope));
				std::cout << "diff_angle: " << diff_angle << std::endl;
				if(diff_angle > M_PI / 2.0){
					diff_angle = M_PI - diff_angle;
				}
				if(diff_angle > SAME_TRAJECTORY_ANGLE_THRESHOLD){
					// robot was turned
					std::cout << "robot is entering new edge" << std::endl;
					linear_trajectories.push_back(trajectory);
					std::cout << "remove curve from last trajectory" << std::endl;
					//remove_curve_from_trajectory(*(linear_trajectories.end() - 2));
					last_slope = slope;
					std::cout << "trajectory was added to trajectories" << std::endl;
					std::cout << "trajectory length: " << Calculation::get_length_of_trajectory(linear_trajectories.back()) << std::endl;
					std::cout << "trajectory angle: " << Calculation::get_angle_from_trajectory(linear_trajectories.back()) << std::endl;
				}else{
					// same line
					std::cout << "robot is curving but on same edge" << std::endl;
					std::copy(trajectory.begin(), trajectory.end(), std::back_inserter(linear_trajectories.back()));
					Calculation::get_slope_from_trajectory(linear_trajectories.back(), last_slope);
					if(tentative_correction_count == 0){
						Eigen::Affine3d diff_correction;
						calculate_affine_transformation_tentatively(diff_correction);
						std::cout << "tentatively corrected line angle: " << Calculation::get_angle_from_trajectory(linear_trajectories.back()) << std::endl;
						odom_correction = diff_correction * odom_correction;
					}
					//last_yaw = estimated_yaw;
					std::cout << "the last of trajectories was extended" << std::endl;
					std::cout << "trajectory length: " << Calculation::get_length_of_trajectory(linear_trajectories.back()) << std::endl;
					std::cout << "trajectory angle: " << Calculation::get_angle_from_trajectory(linear_trajectories.back()) << std::endl;
				}
			}else{
				std::cout << "first edge" << std::endl;
				// first edge
				if(trajectory_length > MIN_LINE_LENGTH){
					Calculation::get_slope_from_trajectory(trajectory, last_slope);
					linear_trajectories.push_back(trajectory);
					//last_yaw = estimated_yaw;
					first_edge_flag = false;
					std::cout << "first edge trajectory was added to trajectories" << std::endl;
					std::cout << "trajectory length: " << Calculation::get_length_of_trajectory(linear_trajectories.back()) << std::endl;
					std::cout << "trajectory angle: " << Calculation::get_angle_from_trajectory(linear_trajectories.back()) << std::endl;
				}
			}
		}else{
			// maybe robot is turning
			std::cout << "robot is curving or going straight longer than threshold, \nbut trajectory size is not larger than MIN_LINE_SIZE" << std::endl;
		}
		trajectory.clear();
		std::cout << "trajectory was cleared" << std::endl;
	}else{
		trajectory.push_back(estimated_pose);
		std::cout << "pose was added to trajectory" << std::endl;
	}
	std::cout << "trajectory.size(): " << trajectory.size() << std::endl;
	std::cout << "trajectory length: " << Calculation::get_length_of_trajectory(trajectory) << std::endl;
	std::cout << "linear_trajectories.size(): " << linear_trajectories.size() << std::endl;
}

void NodeEdgeLocalizer::initialize(void)
{
	nemm.get_edge_from_node_id(INIT_NODE0_ID, INIT_NODE1_ID, estimated_edge);
	estimated_edge.progress = INIT_PROGRESS;
	amsl_navigation_msgs::Node node0;
	nemm.get_node_from_id(estimated_edge.node0_id, node0);
	estimated_pose(0) = node0.point.x + estimated_edge.distance * estimated_edge.progress * cos(estimated_edge.direction);
	estimated_pose(1) = node0.point.y + estimated_edge.distance * estimated_edge.progress * sin(estimated_edge.direction);
	estimated_yaw = INIT_YAW;
	init_estimated_pose = estimated_pose;
	int edge_index = nemm.get_edge_index_from_node_id(INIT_NODE0_ID, INIT_NODE1_ID);
	for(auto& p : particles){
		p.x = estimated_pose(0);
		p.y = estimated_pose(1);
		p.yaw = estimated_yaw;
		p.move(estimated_edge.distance * estimated_edge.progress, estimated_edge.direction);
		p.last_node_x = node0.point.x;
		p.last_node_y = node0.point.y;
		p.weight = 1.0 / (double)PARTICLES_NUM;
		p.current_edge_index = edge_index;
		p.last_edge_index = -1;
	}
	nemm.set_parameters(INIT_NODE0_ID, INIT_NODE1_ID, CONTINUOUS_LINE_THRESHOLD, MIN_LINE_LENGTH);
	init_flag = false;
}

void NodeEdgeLocalizer::correct(void)
{

	int passed_lines_size = nemm.get_passed_line_directions_size();
	std::cout << "passed lines: " <<  passed_lines_size << std::endl;
	std::cout << "correction_count: " << correction_count << std::endl;

	if(passed_lines_size > correction_count && (int)linear_trajectories.size() > correction_count + 1){
		std::cout << "--- correction ---" << std::endl;
		double ratio;
		double direction_diff;
		Eigen::Affine3d diff_correction;
		bool succeeded = calculate_affine_tranformation(correction_count, ratio, direction_diff, diff_correction);

		if(succeeded){
			odom_correction = diff_correction * odom_correction;
			yaw_correction += direction_diff;
			correct_trajectories(correction_count, diff_correction);
			std::cout << "corrected" << std::endl;
			correction_count++;
			std::cout << "corrected line angle: " << Calculation::get_angle_from_trajectory(linear_trajectories.back()) << std::endl;
			tentative_correction_count = POSE_NUM_PCA;
		}else{
			std::cout << "### failed to correct ###" << std::endl;
			std::cout << "### correction reset ###" << std::endl;
			clear_flag = true;
		}
	}
}

bool NodeEdgeLocalizer::calculate_affine_tranformation(const int count, double& ratio, double& direction_diff, Eigen::Affine3d& affine_transformation)
{
	remove_shorter_line_from_trajectories(count);
	double direction_from_odom = Calculation::get_angle_from_trajectory(linear_trajectories[count]);
	// direction_from_odom = Calculation::get_slope_angle(direction_from_odom);
	double direction_from_map = nemm.get_passed_line_direction(count);
	// direction_from_map = Calculation::get_slope_angle(direction_from_map);
	direction_diff = Calculation::pi_2_pi(direction_from_map - direction_from_odom);
	direction_diff = Calculation::get_slope_angle(direction_diff);

	std::cout << "direction from odom: " << direction_from_odom << "[rad]" << std::endl;
	std::cout << "direction from map: " << direction_from_map << "[rad]" << std::endl;
	std::cout << "direction difference: " << direction_diff << "[rad]" << std::endl;

	nemm.show_line_edge_ids();
	// This represents B(i) in paper
	Eigen::Vector3d intersection_point_i;
	std::vector<Eigen::Vector3d> longest_line;
	get_intersection_from_trajectories(*(linear_trajectories.begin() + count), *(linear_trajectories.begin() + count + 1), intersection_point_i);
	std::cout << "B(i): \n" << intersection_point_i << std::endl;
	// This represents N(i) in paper
	Eigen::Vector3d map_node_point_i;
	map_node_point_i = nemm.get_passed_node(count);
	std::cout << "N(i): \n" << map_node_point_i << std::endl;
	// This represents N(i-1) in paper
	Eigen::Vector3d map_node_point_i_1;
	if(count - 1 >= 0){
		map_node_point_i_1 = nemm.get_passed_node(count - 1);
	}else{
		amsl_navigation_msgs::Node init_node;
		nemm.get_node_from_id(INIT_NODE0_ID, init_node);
		map_node_point_i_1 << init_node.point.x, init_node.point.y, 0.0;
	}
	std::cout << "N(i-1): \n" << map_node_point_i_1 << std::endl;

	// distance from B(i) to B(i-1)
	double dist_from_odom = Calculation::get_length_of_trajectory(*(linear_trajectories.begin() + count));
	// distance from N(i) to N(i-1)
	double dist_from_map = (map_node_point_i - map_node_point_i_1).norm();

	if(count){
		ratio = dist_from_odom / dist_from_map;
	}else{
		ratio = 1.0;
	}

	/*
	 * weight has been omitted
	 */

	std::cout << "B(i-1) to B(i): " << dist_from_odom << "[m]" << std::endl;
	std::cout << "N(i-1) to N(i): " << dist_from_map << "[m]" << std::endl;
	std::cout << "ratio: " << ratio << std::endl;
	std::cout << "count: " << count << std::endl;

	double dist_odom_map = (map_node_point_i - intersection_point_i).norm();
	std::cout << "B(i) to N(i): " << dist_odom_map << "[m]" << std::endl;
	if((dist_odom_map < nemm.get_end_of_line_edge_distance())){
		if(fabs(direction_diff) < CORRECTION_REJECTION_ANGLE_DIFFERENCE_THRESHOLD){
			// correction succeeded
			Eigen::Translation<double, 3> t1(map_node_point_i);
			Eigen::Translation<double, 3> t2(-intersection_point_i);
			Eigen::Matrix3d rotation;
			rotation = Eigen::AngleAxisd(direction_diff, Eigen::Vector3d::UnitZ());
			affine_transformation = t1 * rotation * t2;
			std::cout << "affine transformation: \n" << affine_transformation.translation() << "\n" << affine_transformation.rotation().eulerAngles(0,1,2) << std::endl;
			return true;
		}else{
			std::cout << "correction was rejected due to large direction difference between edge and trajectory line" << std::endl;
			return false;
		}
	}else{
		std::cout << "correction was rejected due to too long trajectory line" << std::endl;
		return false;
	}
}

void NodeEdgeLocalizer::calculate_affine_transformation_tentatively(Eigen::Affine3d& affine_transformation)
{
	// current line correction
	std::cout << "# correct tentatively #" << std::endl;
	double direction_from_odom = Calculation::get_angle_from_trajectory(linear_trajectories.back());
	// direction_from_odom = Calculation::get_slope_angle(direction_from_odom);
	amsl_navigation_msgs::Node map_node_point_begin;
	nemm.get_begin_node_of_begin_line_edge(map_node_point_begin);
	amsl_navigation_msgs::Node map_node_point_last;
	nemm.get_end_node_of_last_line_edge(map_node_point_last);
	double direction_from_map = atan2(map_node_point_last.point.y - map_node_point_begin.point.y, map_node_point_last.point.x - map_node_point_begin.point.x);
	// direction_from_map = Calculation::get_slope_angle(direction_from_map);
	double direction_diff = Calculation::pi_2_pi(direction_from_map - direction_from_odom);
	direction_diff = Calculation::get_slope_angle(direction_diff);

	std::cout << "direction from odom: " << direction_from_odom << "[rad]" << std::endl;
	std::cout << "direction from map: " << direction_from_map << "[rad]" << std::endl;
	std::cout << "direction difference: " << direction_diff << "[rad]" << std::endl;

	if(fabs(direction_diff) < CORRECTION_REJECTION_ANGLE_DIFFERENCE_THRESHOLD){
		// This represents B(i) in paper
		Eigen::Vector3d intersection_point_i;
		if(correction_count > 0){
			get_intersection_from_trajectories(*(linear_trajectories.begin() + correction_count - 1), *(linear_trajectories.end() - 1), intersection_point_i);
		}else{
			intersection_point_i = init_estimated_pose;
		}
		std::cout << "B(i): \n" << intersection_point_i << std::endl;
		// This represents N(i) in paper
		nemm.show_line_edge_ids();
		Eigen::Vector3d map_node_point_i;
		map_node_point_i << map_node_point_begin.point.x, map_node_point_begin.point.y, 0.0;
		std::cout << "N(i): \n" << map_node_point_i << std::endl;

		double dist_odom_map = (map_node_point_i - intersection_point_i).norm();
		std::cout << "B(i) to N(i): " << dist_odom_map << "[m]" << std::endl;
		if(dist_odom_map < nemm.get_end_of_line_edge_distance()){
			Eigen::Translation<double, 3> t1(map_node_point_i);
			Eigen::Translation<double, 3> t2(-intersection_point_i);
			Eigen::Matrix3d rotation;
			rotation = Eigen::AngleAxisd(direction_diff, Eigen::Vector3d::UnitZ());
			affine_transformation = t1 * rotation * t2;
			yaw_correction += direction_diff;
			std::cout << "affine transformation: \n" << affine_transformation.translation() << "\n" << affine_transformation.rotation().eulerAngles(0,1,2) << std::endl;
			tentative_correction_count = POSE_NUM_PCA;
			correct_trajectories(linear_trajectories.size() - 1, affine_transformation);
		}else{
			std::cout << "correction was interrupted because B(i) is far away from N(i)" << std::endl;
			tentative_correction_count = POSE_NUM_PCA;
			affine_transformation = Eigen::Affine3d::Identity();
		}
	}else{
		std::cout << "correction was interrupted because the angle difference is large" << std::endl;
		tentative_correction_count = POSE_NUM_PCA;
		affine_transformation = Eigen::Affine3d::Identity();
	}
}

void NodeEdgeLocalizer::get_intersection_from_trajectories(std::vector<Eigen::Vector3d>& trajectory_0, std::vector<Eigen::Vector3d>& trajectory_1, Eigen::Vector3d& intersection_point)
{
	Eigen::Vector2d center_0, center_1, slope_0, slope_1;

	Calculation::get_slope_and_center_from_trajectory(trajectory_0, slope_0, center_0);
	Calculation::get_slope_and_center_from_trajectory(trajectory_1, slope_1, center_1);

	double a0 = slope_0(1) / slope_0(0);
	double a1 = slope_1(1) / slope_1(0);

	double c0 = -a0 * center_0(0) + center_0(1);
	double c1 = -a1 * center_1(0) + center_1(1);

	intersection_point << (-c1 + c0) / (a1 - a0),
						  (-a0 * c1 + a1 * c0) / (a1 - a0),
						  0.0;
}

double NodeEdgeLocalizer::calculate_trajectory_curvature(void)
{
	static int count = 0;
	double x_ave = 0.0;
	double y_ave = 0.0;
	static std::vector<Eigen::Vector3d> pose_buffer(POSE_NUM_PCA, Eigen::Vector3d::Zero());

	// sequential calculation
	static double x_sum = 0.0;
	static double y_sum = 0.0;
	static double xx_sum = 0.0;
	static double yy_sum = 0.0;
	static double xy_sum = 0.0;

	// remove old data
	x_sum -= pose_buffer[count](0);
	y_sum -= pose_buffer[count](1);
	xx_sum -= Calculation::square(pose_buffer[count](0));
	yy_sum -= Calculation::square(pose_buffer[count](1));
	xy_sum -= pose_buffer[count](0) * pose_buffer[count](1);

	// update buffer
	pose_buffer[count] = estimated_pose;

	// add new data
	x_sum += pose_buffer[count](0);
	y_sum += pose_buffer[count](1);
	xx_sum += Calculation::square(pose_buffer[count](0));
	yy_sum += Calculation::square(pose_buffer[count](1));
	xy_sum += pose_buffer[count](0) * pose_buffer[count](1);

	x_ave = x_sum / (double)POSE_NUM_PCA;
	y_ave = y_sum / (double)POSE_NUM_PCA;

	double covariance = xy_sum / (double)POSE_NUM_PCA - x_ave * y_ave;
	Eigen::Matrix2d covariance_matrix;
	covariance_matrix << xx_sum / POSE_NUM_PCA - x_ave * x_ave, covariance,
						 covariance, yy_sum / POSE_NUM_PCA - y_ave * y_ave;
	//std::cout << "covariance matrix: \n" << covariance_matrix << std::endl;

	Eigen::EigenSolver<Eigen::Matrix2d> es(covariance_matrix);
	Eigen::Vector2d values = es.eigenvalues().real();

	double curvature = 100;
	if(values(0) + values(1)){
		curvature = (values(0) < values(1) ? values(0) : values(1)) / (values(0) + values(1));
	}
	count = (count + 1) % POSE_NUM_PCA;

	return curvature;
}

void NodeEdgeLocalizer::publish_pose(void)
{
	nav_msgs::Odometry odom;
	odom.header.frame_id = nemm.get_map_header_frame_id();
	odom.header.stamp = ros::Time::now();
	odom.child_frame_id = robot_frame_id;
	odom.pose.pose.position.x = estimated_pose(0);
	odom.pose.pose.position.y = estimated_pose(1);
	odom.pose.pose.position.z = estimated_pose(2);
	odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(estimated_yaw);

	odom_pub.publish(odom);

	if(ENABLE_TF){
		try{
			tf::Transform map_to_robot;
			tf::poseMsgToTF(odom.pose.pose, map_to_robot);
			tf::Stamped<tf::Pose> robot_to_map(map_to_robot.inverse(), odom_time, robot_frame_id);
			tf::Stamped<tf::Pose> odom_to_map;
			listener.transformPose(odom_frame_id, robot_to_map, odom_to_map);

			broadcaster.sendTransform(tf::StampedTransform(odom_to_map.inverse(), odom.header.stamp, nemm.get_map_header_frame_id(), odom_frame_id));
		}catch(tf::TransformException ex){
			std::cout << ex.what() << std::endl;
		}
	}
	std::cout << "pose was published" << std::endl;
}

void NodeEdgeLocalizer::publish_particles(void)
{
	static geometry_msgs::PoseArray _particles;
	_particles.poses.resize(PARTICLES_NUM);
	_particles.header.frame_id = nemm.get_map_header_frame_id();
	_particles.header.stamp = ros::Time::now();
	for(int i=0;i<PARTICLES_NUM;i++){
		geometry_msgs::Pose p;
		p.position.x = particles[i].x;
		p.position.y = particles[i].y;
		p.orientation = tf::createQuaternionMsgFromYaw(particles[i].yaw);
		_particles.poses[i] = p;
	}
	particles_pub.publish(_particles);
	_particles.poses.clear();
	std::cout << "particles was published" << std::endl;
}

void NodeEdgeLocalizer::particle_filter(int& unique_edge_index, bool& unique_edge_flag)
{
	// unimplemented
	static int resampling_count = 0;
	static std::mt19937 mt{std::random_device{}()};
	std::normal_distribution<> rand(0, NOISE_SIGMA);

	unique_edge_index = -1;
	unique_edge_flag = true;

	std::cout << "robot moved_distance: " << robot_moved_distance << "[m]" << std::endl;
	for(auto& p : particles){
		//std::cout << "--" << std::endl;
		//std::cout << "before move: " << p.x << ", " << p.y << std::endl;
		// move particles
		//std::cout << "last node xy: " << p.last_node_x << ", " << p.last_node_y << std::endl;
		//double current_robot_distance_from_last_node = p.get_distance_from_last_node(estimated_pose(0), estimated_pose(1));
		//std::cout << "robot_moved_distance: " << robot_moved_distance << std::endl;
		p.move(robot_moved_distance + rand(mt), nemm.get_edge_from_index(p.current_edge_index).direction);

		// evaluation
		if(!p.near_node_flag){
			p.evaluate(estimated_yaw);
			/*
			if(robot_moved_distance < 0.1){
				// This particle may have returned to the last node
				if(p.last_edge_index == 0){
					// first edge
					p.near_node_flag = true;
				}
			}
			*/
			if(nemm.get_edge_from_index(p.current_edge_index).distance < p.get_particle_distance_from_last_node()){
				// arrived ???
				//std::cout << "particle arrived at node" << std::endl;
				p.near_node_flag = true;
				amsl_navigation_msgs::Node last_node;
				nemm.get_node_from_id(nemm.get_edge_from_index(p.current_edge_index).node1_id, last_node);
				p.last_node_x = last_node.point.x;
				p.last_node_y = last_node.point.y;
				p.x = p.last_node_x;
				p.y = p.last_node_y;
			}
		}else{
			// go out of range of the last node
			// it should be a constant
			double particle_distance_from_last_node = p.get_particle_distance_from_last_node();
			if(particle_distance_from_last_node > EDGE_DECISION_THRESHOLD){
				//std::cout << "particle put on next edge" << std::endl;
				p.near_node_flag = false;
				p.last_edge_index = p.current_edge_index;
				p.current_edge_index = nemm.get_next_edge_index_from_edge_index(p.current_edge_index, estimated_yaw);
				amsl_navigation_msgs::Node node;
				nemm.get_node_from_id(nemm.get_edge_from_index(p.current_edge_index).node0_id, node);
				p.x = node.point.x;
				p.y = node.point.y;
				// particle's moved distance is more suitable ??
				p.move(particle_distance_from_last_node + rand(mt), nemm.get_edge_from_index(p.current_edge_index).direction);
			}
		}
		//std::cout << "after move: " << p.x << ", " << p.y << std::endl;
	}

	// normalize weight
	double max_weight = 0;
	for(auto p : particles){
		double w = p.weight;
		if(max_weight < w){
			max_weight = w;
		}
	}
	for(auto& p : particles){
		p.weight /= max_weight;
	}

	// edge decision
	const int EDGE_NUM = nemm.get_edge_num();
	std::vector<int> edge_vote_list(EDGE_NUM, 0);
	for(const auto& p : particles){
		if(!p.near_node_flag){
			edge_vote_list[p.current_edge_index]++;
		}
	}
	int max_index = 0;
	for(int i=0;i<EDGE_NUM;i++){
		if(edge_vote_list[i] > edge_vote_list[max_index]){
			max_index = i;
		}
	}
	double votes_ratio = edge_vote_list[max_index] / (double)PARTICLES_NUM;
	std::cout << "max index: " << max_index << std::endl;
	std::cout << "ratio: " << votes_ratio << std::endl;
	if(votes_ratio > EDGE_CERTAIN_THRESHOLD){
		unique_edge_flag = true;
		unique_edge_index = max_index;
	}else{
		unique_edge_flag = false;
	}

	// resampling
	if(resampling_count % RESAMPLING_INTERVAL == 0){
		resampling();
		resampling_count = 0;
	}
	resampling_count++;
	std::cout << "unique edge index: " << unique_edge_index << std::endl;
	std::cout << "unique edge flag: " << unique_edge_flag << std::endl;

	set_particle_to_near_edge(unique_edge_flag, unique_edge_index, particles[0]);
	set_dead_end_particle_to_edge_near_robot(unique_edge_flag, unique_edge_index, particles[0]);
}

void NodeEdgeLocalizer::resampling(void)
{
	std::cout << "-- resampling --" << std::endl;
	static std::mt19937 mt{std::random_device{}()};
	static std::vector<NodeEdgeParticle> copied_particles(PARTICLES_NUM);

	double prob = 0.0;
	double t = 0.0;
	double weight_sum = 0.0;
	for(auto p : particles){
		weight_sum += p.weight;
	}
	std::cout << "particle weight sum before resampling: " << weight_sum << std::endl;

	std::uniform_real_distribution<> rand(0.0, weight_sum);

	for(int j=0;j<PARTICLES_NUM;j++){
		prob = 0.0;
		t = rand(mt);
		for(auto p : particles){
			prob += p.weight;
			if(t <= prob){
				copied_particles[j] = p;
				break;
			}
		}
	}

	std::copy(copied_particles.begin(), copied_particles.end(), particles.begin());
}

void NodeEdgeLocalizer::correct_trajectories(int count, const Eigen::Affine3d& correction)
{
	for(auto line=(linear_trajectories.begin()+count);line!=linear_trajectories.end();line++){
		for(auto& v : *(line)){
			v = correction * v;
		}
	}
}

void NodeEdgeLocalizer::clear(int unique_edge_index)
{
	std::cout << "--- clear ---" << std::endl;;
	nemm.clear(unique_edge_index);
	linear_trajectories.clear();
	clear_flag = false;
}

void NodeEdgeLocalizer::visualize_lines(void)
{
	visualization_msgs::Marker lines_marker;
	lines_marker.header.frame_id = nemm.get_map_header_frame_id();
	lines_marker.header.stamp = ros::Time::now();
	lines_marker.ns = "line_viz";
	lines_marker.action = visualization_msgs::Marker::ADD;
	lines_marker.pose.orientation.w = 1;
	lines_marker.id = 0;
	lines_marker.type = visualization_msgs::Marker::LINE_LIST;
	lines_marker.scale.x = 0.3;
	lines_marker.color.r = 1.0;
	lines_marker.color.g = 0.0;
	lines_marker.color.b = 1.0;
	lines_marker.color.a = 1.0;
	for(auto traj : linear_trajectories){
		double length = Calculation::get_length_of_trajectory(traj);
		Eigen::Vector2d slope;
		Eigen::Vector2d center;
		Calculation::get_slope_and_center_from_trajectory(traj, slope, center);
		double angle = atan2(slope(1), slope(0));
		geometry_msgs::Point p;
		p.x = center(0) + 0.5 * length * cos(angle);
		p.y = center(1) + 0.5 * length * sin(angle);
		p.z = 0.1;
		lines_marker.points.push_back(p);
		p.x = center(0) - 0.5 * length * cos(angle);
		p.y = center(1) - 0.5 * length * sin(angle);
		p.z = 0.1;
		lines_marker.points.push_back(p);
	}
	lines_pub.publish(lines_marker);
}

void NodeEdgeLocalizer::remove_curve_from_trajectory(std::vector<Eigen::Vector3d>& traj)
{
	for(int i=0;i<POSE_NUM_PCA;i++){
		if(Calculation::get_length_of_trajectory(traj) < MIN_LINE_LENGTH){
			return;
		}
		traj.pop_back();
	}
}

void NodeEdgeLocalizer::publish_edge(int unique_edge_index, bool unique_edge_flag)
{
	static amsl_navigation_msgs::Node last_node;
	static Eigen::Vector3d last_node_point;
	if(unique_edge_flag){
		estimated_edge = nemm.get_edge_from_index(unique_edge_index);
		nemm.get_node_from_id(estimated_edge.node0_id, last_node);
		last_node_point << last_node.point.x, last_node.point.y, 0.0;
	}
	double distance_from_last_node = (estimated_pose - last_node_point).norm();
	estimated_edge.progress = distance_from_last_node / estimated_edge.distance;
	edge_pub.publish(estimated_edge);

	visualization_msgs::Marker unique_edge_marker;
	unique_edge_marker.header.frame_id = nemm.get_map_header_frame_id();
	unique_edge_marker.header.stamp = ros::Time::now();
	unique_edge_marker.ns = "unique_edge_marker";
	unique_edge_marker.id = 0;
	unique_edge_marker.color.a = 0.3;
	unique_edge_marker.color.g = 1.0;
	unique_edge_marker.pose.orientation = tf::createQuaternionMsgFromYaw(0);
	if(unique_edge_flag){
		unique_edge_marker.action = visualization_msgs::Marker::ADD;
		unique_edge_marker.type = visualization_msgs::Marker::LINE_STRIP;
		unique_edge_marker.scale.x = 1.0;
		amsl_navigation_msgs::Node n;
		nemm.get_node_from_id(estimated_edge.node0_id, n);
		unique_edge_marker.points.push_back(n.point);
		nemm.get_node_from_id(estimated_edge.node1_id, n);
		unique_edge_marker.points.push_back(n.point);
		edge_marker_pub.publish(unique_edge_marker);
	}else{
		unique_edge_marker.action = visualization_msgs::Marker::ADD;
		unique_edge_marker.type = visualization_msgs::Marker::CYLINDER;
		unique_edge_marker.scale.x = 4.0;
		unique_edge_marker.scale.y = 4.0;
		unique_edge_marker.scale.z = 0.1;
		amsl_navigation_msgs::Node n;
		nemm.get_end_node_of_last_line_edge(n);
		unique_edge_marker.pose.position = n.point;
		edge_marker_pub.publish(unique_edge_marker);
	}

	std::cout << "estimated_edge: \n" << estimated_edge << std::endl;
}

void NodeEdgeLocalizer::publish_odom_tf(Eigen::Vector3d& odom, double yaw)
{
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(odom(0), odom(1), odom(2)));
	tf::Quaternion q;
	q.setRPY(0, 0, yaw);
	transform.setRotation(q);
	odom_time = ros::Time::now();
	tf::StampedTransform odom_tf(transform, odom_time, odom_frame_id, robot_frame_id);
	broadcaster.sendTransform(odom_tf);
}

void NodeEdgeLocalizer::remove_shorter_line_from_trajectories(const int count)
{
	std::cout << "remove shorter line from trajectories" << std::endl;
	int longest_line_index = -1;
	double longest_length = 0;
	int size = linear_trajectories.size();
	for(int i=count;i<size-1;i++){
		double length = Calculation::get_length_of_trajectory(linear_trajectories[i]);
		std::cout << "length of line[" << i << "] is " << length << "[m]" << std::endl;
		if(length > longest_length){
			longest_length = length;
			longest_line_index = i;
		}
	}
	std::cout << "the longest line index: " << longest_line_index << std::endl;
	int index = 0;
	for(auto line=(linear_trajectories.begin()+count);line!=linear_trajectories.end()-1;){
		double length = Calculation::get_length_of_trajectory(*line);
		if(length < longest_length){
			line = linear_trajectories.erase(line);
			std::cout << "line[" << index << "] was deleted" << std::endl;
		}else{
			line++;
		}
		index++;
	}
}

void NodeEdgeLocalizer::set_particle_to_near_edge(bool unique_edge_flag, int unique_edge_index, NodeEdgeParticle& p)
{
	if(unique_edge_flag){
		std::vector<amsl_navigation_msgs::Edge> candidate_edges;
		nemm.get_candidate_edges(estimated_yaw, unique_edge_index, candidate_edges);
		int edge_num = candidate_edges.size();
		amsl_navigation_msgs::Node node0;
		nemm.get_node_from_id(candidate_edges[0].node0_id, node0);
		if(edge_num > 0){
			double min_distance = 1e6;
			int min_index = 0;
			for(int i=0;i<edge_num;i++){
				amsl_navigation_msgs::Node node1;
				nemm.get_node_from_id(candidate_edges[i].node1_id, node1);
				double distance = 1e6;
				if(node1.point.x - node0.point.x != 0.0){
					// ax+by+c=0
					double a = (node1.point.y - node0.point.y) / (node1.point.x - node0.point.x);
					double b = -1;
					double c = node0.point.y - a * node0.point.x;
					distance = fabs(a * estimated_pose(0) + b * estimated_pose(1) + c) / sqrt(Calculation::square(a) + Calculation::square(b));
					std::cout << "distance to edge(" << node0.id << ", " << node1.id << "): " << distance << "[m]" << std::endl;
				}else{
					distance = fabs(estimated_pose(0) - node1.point.x);
					std::cout << "distance to edge(" << node0.id << ", " << node1.id << "): " << distance << "[m]" << std::endl;
				}
				if(min_distance > distance){
					min_distance = distance;
					min_index = i;
				}
			}
			double particle_distance_from_last_node = p.get_particle_distance_from_last_node();
			p.last_edge_index = p.current_edge_index;
			p.current_edge_index = nemm.get_edge_index_from_node_id(candidate_edges[min_index].node0_id, candidate_edges[min_index].node1_id);
			p.x = p.last_node_x = node0.point.x;
			p.y = p.last_node_y = node0.point.y;
			p.move(particle_distance_from_last_node, nemm.get_edge_from_index(p.current_edge_index).direction);
			std::cout << "set particle[0] to edge(" << node0.id << ", " << nemm.get_edge_from_index(p.current_edge_index).node1_id << ")" << std::endl;
		}
	}
}

void NodeEdgeLocalizer::set_dead_end_particle_to_edge_near_robot(bool unique_edge_flag, int unique_edge_index, NodeEdgeParticle& p)
{
	double weight_sum = 0.0;

	for(auto particle : particles){
		weight_sum += particle.weight;
	}
	if(weight_sum / PARTICLES_NUM < EDGE_DECISION_THRESHOLD){
		amsl_navigation_msgs::Edge edge_near_robot;
		nemm.get_edge_from_estimated_pose(estimated_pose(0), estimated_pose(1), estimated_yaw, edge_near_robot);
		double particle_distance_from_last_node = p.get_particle_distance_from_last_node();
		p.last_edge_index = p.current_edge_index;
		p.current_edge_index = nemm.get_edge_index_from_node_id(edge_near_robot.node0_id, edge_near_robot.node1_id);
		amsl_navigation_msgs::Node node0;
		nemm.get_node_from_id(edge_near_robot.node0_id, node0);
		p.x = p.last_node_x = node0.point.x;
		p.y = p.last_node_y = node0.point.y;
		p.move(particle_distance_from_last_node, nemm.get_edge_from_index(p.current_edge_index).direction);
		std::cout << "set particle[0] to edge(" << edge_near_robot.node0_id << ", " << edge_near_robot.node1_id << ")" << std::endl;
	}
}