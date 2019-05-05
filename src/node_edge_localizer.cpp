#include <random>

#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

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

class NodeEdgeLocalizer
{
public:
	NodeEdgeLocalizer(void);

	void map_callback(const amsl_navigation_msgs::NodeEdgeMapConstPtr&);
	void odom_callback(const nav_msgs::OdometryConstPtr&);
	void process(void);
	void clustering_trajectories(void);
	void get_node_from_id(int, amsl_navigation_msgs::Node&);
	void get_edge_from_node_id(int, int, amsl_navigation_msgs::Edge&);
	int get_edge_index_from_node_id(int, int);
	double pi_2_pi(double);
	void initialize(void);
	double get_curvature_from_trajectory(std::vector<Eigen::Vector3d>&);
	double get_angle_from_trajectory(std::vector<Eigen::Vector3d>&);
	void get_slope_from_trajectory(std::vector<Eigen::Vector3d>&, Eigen::Vector2d&);
	void get_slope_and_center_from_trajectory(std::vector<Eigen::Vector3d>&, Eigen::Vector2d&, Eigen::Vector2d&);
	void calculate_pca(std::vector<Eigen::Vector3d>&, Eigen::Vector2d&, Eigen::Matrix2d&);
	void calculate_pca(std::vector<Eigen::Vector3d>&, Eigen::Vector2d&, Eigen::Matrix2d&, Eigen::Vector2d&);
	void correct(void);
	void calculate_affine_tranformation(const int, double&, double&, Eigen::Affine3d&);
	void calculate_affine_transformation_tentatively(Eigen::Affine3d&);
	void get_intersection_from_trajectories(std::vector<Eigen::Vector3d>&, std::vector<Eigen::Vector3d>&, Eigen::Vector3d&);
	double get_angle_from_lines(Eigen::Vector3d&, Eigen::Vector3d&, Eigen::Vector3d&, Eigen::Vector3d&);
	double get_length_of_trajectory(std::vector<Eigen::Vector3d>&);
	double square(double);
	int get_index_from_id(int);
	double calculate_trajectory_curvature(void);
	void publish_pose(void);
	void publish_particles(void);
	void particle_filter(int&, bool&);
	void resampling(void);
	int get_next_edge_index_from_edge_index(int);
	void manage_passed_edge(int);
	void correct_trajectories(int, const Eigen::Affine3d&); 
	void clear(int);
	void visualize_lines(void);
	void remove_curve_from_trajectory(std::vector<Eigen::Vector3d>&);
	void publish_edge(int, bool);
	double get_slope_angle(double);

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
	double LINE_EDGE_RATIO_THRESHOLD;

	ros::NodeHandle nh;
	ros::NodeHandle private_nh;

	ros::Publisher pose_pub;
	ros::Publisher edge_pub;
	ros::Publisher odom_pub;
	ros::Publisher particles_pub;
	ros::Publisher lines_pub;
	ros::Subscriber map_sub;
	ros::Subscriber odom_sub;

	tf::TransformListener listener;
	tf::TransformBroadcaster broadcaster;

	amsl_navigation_msgs::NodeEdgeMap map;
	amsl_navigation_msgs::Edge estimated_edge;
	bool map_subscribed;
	bool odom_updated;
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

	// for correction
	std::vector<Eigen::Vector3d> passed_nodes;
	std::vector<double> passed_line_directions;
	int begin_line_edge_index;
	int end_line_edge_index;
	int last_line_edge_index;

	int tentative_correction_count;// count down
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
	map_sub = nh.subscribe("/node_edge_map", 1, &NodeEdgeLocalizer::map_callback, this);
	odom_sub = nh.subscribe("/odom/complement", 1 ,&NodeEdgeLocalizer::odom_callback, this);
	pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/estimated_pose/pose", 1);
	edge_pub = nh.advertise<amsl_navigation_msgs::Edge>("/estimated_pose/edge", 1);
	odom_pub = nh.advertise<nav_msgs::Odometry>("/estimated_pose/odom", 1);
	particles_pub = nh.advertise<geometry_msgs::PoseArray>("/estimated_pose/particles", 1);
	lines_pub = nh.advertise<visualization_msgs::Marker>("/passed_lines/viz", 1);

	private_nh.param("HZ", HZ, {20});
	private_nh.param("INIT_NODE0_ID", INIT_NODE0_ID, {0});
	private_nh.param("INIT_NODE1_ID", INIT_NODE1_ID, {1});
	private_nh.param("INIT_PROGRESS", INIT_PROGRESS, {0.0});
	private_nh.param("INIT_YAW", INIT_YAW, {0.0});
	private_nh.param("CURVATURE_THRESHOLD", CURVATURE_THRESHOLD, {0.01});
	private_nh.param("POSE_NUM_PCA", POSE_NUM_PCA, {30});
	private_nh.param("MIN_LINE_SIZE", MIN_LINE_SIZE, {30});
	private_nh.param("MIN_LINE_LENGTH", MIN_LINE_LENGTH, {3.0});
	private_nh.param("ENABLE_TF", ENABLE_TF, {false});
	private_nh.param("USE_ORIENTATION_Z_AS_YAW", USE_ORIENTATION_Z_AS_YAW, {false});
	private_nh.param("PARTICLES_NUM", PARTICLES_NUM, {1000});
	private_nh.param("NOISE_SIGMA", NOISE_SIGMA, {0.05});
	private_nh.param("EDGE_DECISION_THRESHOLD", EDGE_DECISION_THRESHOLD, {0.5});
	private_nh.param("SAME_TRAJECTORY_ANGLE_THRESHOLD", SAME_TRAJECTORY_ANGLE_THRESHOLD, {M_PI/6.0});
	private_nh.param("CONTINUOUS_LINE_THRESHOLD", CONTINUOUS_LINE_THRESHOLD, {M_PI/7.0});
	private_nh.param("LINE_EDGE_RATIO_THRESHOLD", LINE_EDGE_RATIO_THRESHOLD, {0.5});

	map_subscribed = false;
	odom_updated = false;
	init_flag = true;
	clear_flag = false;
	first_edge_flag = true;
	robot_frame_id = "base_link";
	odom_frame_id = "odom";
	particles.resize(PARTICLES_NUM);
	odom_correction = Eigen::Affine3d::Identity();
	robot_moved_distance = 0.0;
	tentative_correction_count = POSE_NUM_PCA;

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
	std::cout << "LINE_EDGE_RATIO_THRESHOLD: " << LINE_EDGE_RATIO_THRESHOLD << std::endl;
}

void NodeEdgeLocalizer::map_callback(const amsl_navigation_msgs::NodeEdgeMapConstPtr& msg)
{
	map = *msg;
	map_subscribed = true;
}

void NodeEdgeLocalizer::odom_callback(const nav_msgs::OdometryConstPtr& msg)
{
	std::cout << "--- odom calback ---" << std::endl;
	if(!init_flag){
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
		estimated_yaw = odom_yaw + INIT_YAW;
		estimated_yaw = pi_2_pi(estimated_yaw);

		Eigen::Vector3d odom_pose;
		odom_pose << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
		if(first_odom_callback_flag){
			first_odom_pose = odom_pose;
			std::cout << "first odom pose: \n" << first_odom_pose << std::endl;
		}
		odom_pose -= first_odom_pose;
		odom_pose << odom_pose(0) * cos(-first_odom_yaw) - odom_pose(1) * sin(-first_odom_yaw),
					 odom_pose(0) * sin(-first_odom_yaw) + odom_pose(1) * cos(-first_odom_yaw),
					 0.0;
		Eigen::Vector3d odom_to_map;
		odom_to_map << odom_pose(0) * cos(INIT_YAW) - odom_pose(1) * sin(INIT_YAW),
					   odom_pose(0) * sin(INIT_YAW) + odom_pose(1) * cos(INIT_YAW),
					   0.0;
		estimated_pose = odom_correction * (odom_to_map + init_estimated_pose);

		std::cout << "odom_pose: \n" << odom_pose << std::endl;
		std::cout << "odom_yaw: \n" << odom_yaw << std::endl;
		std::cout << "odom_correction: \n" << odom_correction.matrix() << std::endl;
		std::cout << "yaw_correction: " << yaw_correction << "[rad]" << std::endl;
		std::cout << "estimated_pose: \n" << estimated_pose << std::endl;
		std::cout << "estimated_yaw: " << estimated_yaw << "[rad]" << std::endl;
		robot_frame_id = msg->child_frame_id;
		odom_frame_id = msg->header.frame_id;
		if(first_odom_callback_flag){
			first_odom_callback_flag = false;
		}else{
			odom_updated = true;
			// robot moveed distance for particle
			Eigen::Vector3d move_vector = estimated_pose - last_estimated_pose;
			robot_moved_distance = move_vector.norm();
			double moved_direction = atan2(move_vector(1), move_vector(0));
			double diff_yaw_and_moved_direction = estimated_yaw - moved_direction;
			diff_yaw_and_moved_direction = pi_2_pi(diff_yaw_and_moved_direction);
			robot_moved_distance *= cos(diff_yaw_and_moved_direction);
			if(fabs(diff_yaw_and_moved_direction) < M_PI / 2.0){
				// forward
				robot_moved_distance = robot_moved_distance;
			}else{
				// back
				robot_moved_distance = -robot_moved_distance;
			}
		}
	}else{
		std::cout << "not initialized !!!" << std::endl;
	}
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
					manage_passed_edge(unique_edge_index);
				}
				std::cout << "--- calculate correction ---" << std::endl;
				correct();
				if(clear_flag){
					clear(unique_edge_index);
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
	double trajectory_length = get_length_of_trajectory(trajectory);
	std::cout << "trajectory length: " << trajectory_length << std::endl;
	if(trajectory_curvature > CURVATURE_THRESHOLD || trajectory_length > MIN_LINE_LENGTH){
		if(trajectory.size() > MIN_LINE_SIZE){
			static Eigen::Vector2d last_slope;
			static double last_yaw = 0;
			if(!first_edge_flag){
				Eigen::Vector2d slope;
				// get slope of current trajectory
				std::cout << "trajectory angle: " << get_angle_from_trajectory(trajectory) << std::endl;
				get_slope_from_trajectory(trajectory, slope);
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
					remove_curve_from_trajectory(*(linear_trajectories.end() - 2));
					last_slope = slope;
					std::cout << "trajectory was added to trajectories" << std::endl;
					std::cout << "trajectory length: " << get_length_of_trajectory(linear_trajectories.back()) << std::endl;
					std::cout << "trajectory angle: " << get_angle_from_trajectory(linear_trajectories.back()) << std::endl;
				}else{
					// same line 
					std::cout << "robot is curving but on same edge" << std::endl;
					std::copy(trajectory.begin(), trajectory.end(), std::back_inserter(linear_trajectories.back()));
					get_slope_from_trajectory(linear_trajectories.back(), last_slope);
					if(tentative_correction_count == 0){
						Eigen::Affine3d diff_correction;
						calculate_affine_transformation_tentatively(diff_correction);
						std::cout << "tentatively corrected line angle: " << get_angle_from_trajectory(linear_trajectories.back()) << std::endl;
						odom_correction = diff_correction * odom_correction;
					}
					last_yaw = estimated_yaw;
					std::cout << "the last of trajectories was extended" << std::endl;
					std::cout << "trajectory length: " << get_length_of_trajectory(linear_trajectories.back()) << std::endl;
					std::cout << "trajectory angle: " << get_angle_from_trajectory(linear_trajectories.back()) << std::endl;
				}
			}else{
				// first edge
				if(trajectory_length > MIN_LINE_LENGTH){
					get_slope_from_trajectory(trajectory, last_slope);
					linear_trajectories.push_back(trajectory);
					last_yaw = estimated_yaw;
					first_edge_flag = false;
					std::cout << "first edge trajectory was added to trajectories" << std::endl;
					std::cout << "trajectory length: " << get_length_of_trajectory(linear_trajectories.back()) << std::endl;
					std::cout << "trajectory angle: " << get_angle_from_trajectory(linear_trajectories.back()) << std::endl;
				}
			}
		}else{
			// maybe robot is turning
		}
		trajectory.clear();
		std::cout << "trajectory was cleared" << std::endl;
	}else{
		trajectory.push_back(estimated_pose);
		std::cout << "pose was added to trajectory" << std::endl;
	}
	std::cout << "trajectory.size(): " << trajectory.size() << std::endl;
	std::cout << "trajectory length: " << get_length_of_trajectory(trajectory) << std::endl;
	std::cout << "linear_trajectories.size(): " << linear_trajectories.size() << std::endl;
}

void NodeEdgeLocalizer::get_node_from_id(int id, amsl_navigation_msgs::Node& node)
{
	for(auto n : map.nodes){
		if(n.id == id){
			node = n;
			return;
		}
	}
}

void NodeEdgeLocalizer::get_edge_from_node_id(int node0_id, int node1_id, amsl_navigation_msgs::Edge& edge)
{
	for(auto e : map.edges){
		if(e.node0_id == node0_id && e.node1_id == node1_id){
			edge = e;
			return;
		}
	}
}

int NodeEdgeLocalizer::get_edge_index_from_node_id(int node0_id, int node1_id)
{
	int index = 0;
	for(auto e : map.edges){
		if(e.node0_id == node0_id && e.node1_id == node1_id){
			return index;
		}
		index++;
	}
}

double NodeEdgeLocalizer::pi_2_pi(double angle)
{
	return atan2(sin(angle), cos(angle));
}

void NodeEdgeLocalizer::initialize(void)
{
	get_edge_from_node_id(INIT_NODE0_ID, INIT_NODE1_ID, estimated_edge);
	estimated_edge.progress = INIT_PROGRESS;
	amsl_navigation_msgs::Node node0;
	get_node_from_id(estimated_edge.node0_id, node0);
	estimated_pose(0) = node0.point.x + estimated_edge.distance * estimated_edge.progress * cos(estimated_edge.direction);
	estimated_pose(1) = node0.point.y + estimated_edge.distance * estimated_edge.progress * sin(estimated_edge.direction);
	estimated_yaw = INIT_YAW;
	init_estimated_pose = estimated_pose;
	int edge_index = get_edge_index_from_node_id(INIT_NODE0_ID, INIT_NODE1_ID);
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
	begin_line_edge_index = get_edge_index_from_node_id(INIT_NODE0_ID, INIT_NODE1_ID);
	end_line_edge_index = get_edge_index_from_node_id(INIT_NODE0_ID, INIT_NODE1_ID);
	last_line_edge_index = get_edge_index_from_node_id(INIT_NODE0_ID, INIT_NODE1_ID);
	init_flag = false;
}

double NodeEdgeLocalizer::get_curvature_from_trajectory(std::vector<Eigen::Vector3d>& traj)
{
	Eigen::Vector2d eigen_values;
	Eigen::Matrix2d eigen_vectors;
	calculate_pca(traj, eigen_values, eigen_vectors);
	double min_value = (eigen_values(0) < eigen_values(1)) ? eigen_values(0) : eigen_values(1); 
	double curvature = min_value / (eigen_values(0) + eigen_values(1));
	return curvature;
}

double NodeEdgeLocalizer::get_angle_from_trajectory(std::vector<Eigen::Vector3d>& traj)
{
	Eigen::Vector2d eigen_values;
	Eigen::Matrix2d eigen_vectors;
	calculate_pca(traj, eigen_values, eigen_vectors);
	double larger_index = (eigen_values(0) > eigen_values(1)) ? 0 : 1; 
	Eigen::Vector2d larger_vector = eigen_vectors.col(larger_index);
	double angle = atan2(larger_vector(1), larger_vector(0));
	return angle;
}

void NodeEdgeLocalizer::get_slope_from_trajectory(std::vector<Eigen::Vector3d>& traj, Eigen::Vector2d& slope)
{
	Eigen::Vector2d eigen_values;
	Eigen::Matrix2d eigen_vectors;
	calculate_pca(traj, eigen_values, eigen_vectors);
	double larger_index = (eigen_values(0) > eigen_values(1)) ? 0 : 1; 
	Eigen::Vector2d larger_vector = eigen_vectors.col(larger_index);
	// the first principal component
	slope = larger_vector;
}

void NodeEdgeLocalizer::get_slope_and_center_from_trajectory(std::vector<Eigen::Vector3d>& traj, Eigen::Vector2d& slope, Eigen::Vector2d& center)
{
	Eigen::Vector2d eigen_values;
	Eigen::Matrix2d eigen_vectors;
	calculate_pca(traj, eigen_values, eigen_vectors, center);
	double larger_index = (eigen_values(0) > eigen_values(1)) ? 0 : 1; 
	Eigen::Vector2d larger_vector = eigen_vectors.col(larger_index);
	// the first principal component
	slope = larger_vector;
}

void NodeEdgeLocalizer::calculate_pca(std::vector<Eigen::Vector3d>& traj, Eigen::Vector2d& eigen_values, Eigen::Matrix2d& eigen_vectors)
{
	// principal component analysis
	double size = traj.size();
	double ave_x = 0;
	double ave_y = 0;
	for(auto& point : traj){
		ave_x += point(0);	
		ave_y += point(1);	
	}
	ave_x /= size;
	ave_y /= size;
	double sigma_xx = 0;
	double sigma_xy = 0;
	double sigma_yy = 0;
	for(auto& point : traj){
		sigma_xx += (point(0) - ave_x) * (point(0) - ave_x); 
		sigma_xy += (point(0) - ave_x) * (point(1) - ave_y); 
		sigma_yy += (point(1) - ave_y) * (point(1) - ave_y); 
	}
	Eigen::Matrix2d cov_mat;
	cov_mat << sigma_xx, sigma_xy,
			   sigma_xy, sigma_yy; 
	Eigen::EigenSolver<Eigen::Matrix2d> es(cov_mat);
	eigen_values = es.eigenvalues().real();
	eigen_vectors = es.eigenvectors().real();
}

void NodeEdgeLocalizer::calculate_pca(std::vector<Eigen::Vector3d>& traj, Eigen::Vector2d& eigen_values, Eigen::Matrix2d& eigen_vectors, Eigen::Vector2d& center)
{
	// principal component analysis
	double size = traj.size();
	double ave_x = 0;
	double ave_y = 0;
	for(auto& point : traj){
		ave_x += point(0);	
		ave_y += point(1);	
	}
	ave_x /= size;
	ave_y /= size;
	double sigma_xx = 0;
	double sigma_xy = 0;
	double sigma_yy = 0;
	for(auto& point : traj){
		sigma_xx += (point(0) - ave_x) * (point(0) - ave_x); 
		sigma_xy += (point(0) - ave_x) * (point(1) - ave_y); 
		sigma_yy += (point(1) - ave_y) * (point(1) - ave_y); 
	}
	Eigen::Matrix2d cov_mat;
	cov_mat << sigma_xx, sigma_xy,
			   sigma_xy, sigma_yy; 
	Eigen::EigenSolver<Eigen::Matrix2d> es(cov_mat);
	eigen_values = es.eigenvalues().real();
	eigen_vectors = es.eigenvectors().real();
	center << ave_x, ave_y;
}

void NodeEdgeLocalizer::correct(void)
{
	static int correction_count = 0;

	std::cout << "passed lines: " << passed_line_directions.size() << std::endl;
	std::cout << "correction_count: " << correction_count << std::endl;

	if(passed_line_directions.size() > correction_count && linear_trajectories.size() > correction_count + 1){
		std::cout << "--- correction ---" << std::endl;
		double ratio;
		double direction_diff;
		Eigen::Affine3d diff_correction;
		calculate_affine_tranformation(correction_count, ratio, direction_diff, diff_correction);

		if(ratio > LINE_EDGE_RATIO_THRESHOLD){
			if(direction_diff < M_PI / 15.0){
				odom_correction = diff_correction * odom_correction;
				yaw_correction += direction_diff;
				correct_trajectories(correction_count, diff_correction);
				std::cout << "corrected" << std::endl;
				correction_count++;
				std::cout << "corrected line angle: " << get_angle_from_trajectory(linear_trajectories.back()) << std::endl;
				tentative_correction_count = POSE_NUM_PCA;
			}else{
				std::cout << "### failed to correct ###" << std::endl;
				std::cout << "### correction reset ###" << std::endl;
				clear_flag = true;
			}
		}else{
			std::cout << "### correction update was rejected because of very short line ###" << std::endl;
			std::cout << "### line [" << correction_count << "] was deleted ###" << std::endl;
			linear_trajectories.erase(linear_trajectories.begin() + correction_count);
		}
	}
}

void NodeEdgeLocalizer::calculate_affine_tranformation(const int count, double& ratio, double& direction_diff, Eigen::Affine3d& affine_transformation)
{
	double direction_from_odom = get_angle_from_trajectory(linear_trajectories[count]);
	// direction_from_odom = get_slope_angle(direction_from_odom);
	double direction_from_map = passed_line_directions[count];
	// direction_from_map = get_slope_angle(direction_from_map);
	direction_diff = pi_2_pi(direction_from_map - direction_from_odom);
	direction_diff = get_slope_angle(direction_diff);

	std::cout << "direction from odom: " << direction_from_odom << "[rad]" << std::endl;
	std::cout << "direction from map: " << direction_from_map << "[rad]" << std::endl;
	std::cout << "direction difference: " << direction_diff << "[rad]" << std::endl;

	std::cout << begin_line_edge_index << ", " << end_line_edge_index << ", " << last_line_edge_index << std::endl;
	// This represents B(i) in paper
	Eigen::Vector3d intersection_point_i;
	get_intersection_from_trajectories(*(linear_trajectories.begin() + count), *(linear_trajectories.begin() + count + 1), intersection_point_i);
	std::cout << "B(i): \n" << intersection_point_i << std::endl;
	// This represents N(i) in paper
	Eigen::Vector3d map_node_point_i;
	map_node_point_i = passed_nodes[count];
	std::cout << "N(i): \n" << map_node_point_i << std::endl;
	// This represents N(i-1) in paper
	Eigen::Vector3d map_node_point_i_1;
	if(count - 1 >= 0){
		map_node_point_i_1 = passed_nodes[count - 1];
	}else{
		amsl_navigation_msgs::Node init_node;
		init_node = map.nodes[get_index_from_id(INIT_NODE0_ID)];
		map_node_point_i_1 << init_node.point.x, init_node.point.y, 0.0;
	}
	std::cout << "N(i-1): \n" << map_node_point_i_1 << std::endl;

	// distance from B(i) to B(i-1)
	double dist_from_odom = get_length_of_trajectory(*(linear_trajectories.begin() + count));
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

	// Eigen::Translation<double, 3> t1(intersection_point_i - map_node_point_i);
	Eigen::Translation<double, 3> t1(map_node_point_i);
	Eigen::Translation<double, 3> t2(-intersection_point_i);
	Eigen::Matrix3d rotation;
	rotation = Eigen::AngleAxisd(direction_diff, Eigen::Vector3d::UnitZ());
	affine_transformation = t1 * rotation * t2;
	std::cout << "affine transformation: \n" << affine_transformation.translation() << "\n" << affine_transformation.rotation().eulerAngles(0,1,2) << std::endl;
}

void NodeEdgeLocalizer::calculate_affine_transformation_tentatively(Eigen::Affine3d& affine_transformation)
{
	// current line correction
	std::cout << "# correct tentatively #" << std::endl;
	double direction_from_odom = get_angle_from_trajectory(linear_trajectories.back());
	// direction_from_odom = get_slope_angle(direction_from_odom);
	amsl_navigation_msgs::Node map_node_point_begin;
	map_node_point_begin = map.nodes[get_index_from_id(map.edges[begin_line_edge_index].node0_id)];
	amsl_navigation_msgs::Node map_node_point_last;
	map_node_point_last = map.nodes[get_index_from_id(map.edges[last_line_edge_index].node1_id)];
	double direction_from_map = atan2(map_node_point_last.point.y - map_node_point_begin.point.y, map_node_point_last.point.x - map_node_point_begin.point.x);
	// direction_from_map = get_slope_angle(direction_from_map);
	double direction_diff = pi_2_pi(direction_from_map - direction_from_odom);
	direction_diff = get_slope_angle(direction_diff);

	std::cout << "direction from odom: " << direction_from_odom << "[rad]" << std::endl;
	std::cout << "direction from map: " << direction_from_map << "[rad]" << std::endl;
	std::cout << "direction difference: " << direction_diff << "[rad]" << std::endl;

	// This represents B(i) in paper
	Eigen::Vector3d intersection_point_i;
	if(linear_trajectories.size() > 1){
		get_intersection_from_trajectories(*(linear_trajectories.end() - 2), *(linear_trajectories.end() - 1), intersection_point_i);
	}else{
		intersection_point_i = init_estimated_pose;
	}
	std::cout << "B(i): \n" << intersection_point_i << std::endl;
	// This represents N(i) in paper
	Eigen::Vector3d map_node_point_i;
	std::cout << begin_line_edge_index << ", " << end_line_edge_index << ", " << last_line_edge_index << std::endl;
	map_node_point_i << map_node_point_begin.point.x, map_node_point_begin.point.y, 0.0;
	std::cout << "N(i): \n" << map_node_point_i << std::endl;

	Eigen::Translation<double, 3> t1(map_node_point_i);
	Eigen::Translation<double, 3> t2(-intersection_point_i);
	Eigen::Matrix3d rotation;
	rotation = Eigen::AngleAxisd(direction_diff, Eigen::Vector3d::UnitZ());
	affine_transformation = t1 * rotation * t2;
	yaw_correction += direction_diff;
	std::cout << "affine transformation: \n" << affine_transformation.translation() << "\n" << affine_transformation.rotation().eulerAngles(0,1,2) << std::endl;
	tentative_correction_count = POSE_NUM_PCA;
	correct_trajectories(linear_trajectories.size() - 1, affine_transformation);
}

void NodeEdgeLocalizer::get_intersection_from_trajectories(std::vector<Eigen::Vector3d>& trajectory_0, std::vector<Eigen::Vector3d>& trajectory_1, Eigen::Vector3d& intersection_point)
{
	Eigen::Vector2d center_0, center_1, slope_0, slope_1;

	get_slope_and_center_from_trajectory(trajectory_0, slope_0, center_0);
	get_slope_and_center_from_trajectory(trajectory_1, slope_1, center_1);

	double a0 = slope_0(1) / slope_0(0);
	double a1 = slope_1(1) / slope_1(0);

	double c0 = -a0 * center_0(0) + center_0(1);
	double c1 = -a1 * center_1(0) + center_1(1);

	intersection_point << (-c1 + c0) / (a1 - a0),
					      (-a0 * c1 + a1 * c0) / (a1 - a0),
						  0.0;
}

double NodeEdgeLocalizer::get_angle_from_lines(Eigen::Vector3d& line0_p0, Eigen::Vector3d& line0_p1, Eigen::Vector3d& line1_p0, Eigen::Vector3d& line1_p1)
{
	double v0_x = line0_p0(0) - line0_p1(0);
	double v0_y = line0_p0(1) - line0_p1(1);
	double v1_x = line1_p0(0) - line1_p1(0);
	double v1_y = line1_p0(1) - line1_p1(1);
	return acos((v0_x * v1_x + v0_y * v1_y) / (sqrt(v0_x * v0_x + v0_y * v0_y) * sqrt(v1_x * v1_x + v1_y * v1_y)));
}

double NodeEdgeLocalizer::get_length_of_trajectory(std::vector<Eigen::Vector3d>& traj)
{
	if(traj.empty()){
		return 0;
	}
	Eigen::Vector3d p0 = *(traj.begin());
	Eigen::Vector3d p1 = *(traj.end() - 1);
	Eigen::Vector3d diff = p1 - p0;
	return sqrt(square(diff(0)) + square(diff(1)));
}

double NodeEdgeLocalizer::square(double value)
{
	return value * value;
}

int NodeEdgeLocalizer::get_index_from_id(int id)
{
	int i = 0;
	for(auto n : map.nodes){
		if(n.id == id){
			return i;
		}
		i++;
	}
	return -1;
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
	xx_sum -= square(pose_buffer[count](0));
	yy_sum -= square(pose_buffer[count](1));
	xy_sum -= pose_buffer[count](0) * pose_buffer[count](1);

	// update buffer
	pose_buffer[count] = estimated_pose;

	// add new data
	x_sum += pose_buffer[count](0);
	y_sum += pose_buffer[count](1);
	xx_sum += square(pose_buffer[count](0));
	yy_sum += square(pose_buffer[count](1));
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
	odom.header.frame_id = map.header.frame_id;
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
			tf::Stamped<tf::Pose> robot_to_map(map_to_robot.inverse(), odom.header.stamp, robot_frame_id);
			tf::Stamped<tf::Pose> odom_to_map;
			listener.transformPose(odom.header.frame_id, robot_to_map, odom_to_map);

			broadcaster.sendTransform(tf::StampedTransform(odom_to_map.inverse(), odom.header.stamp, map.header.frame_id, odom.header.frame_id));
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
	_particles.header.frame_id = map.header.frame_id;
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

	for(auto& p : particles){
		//std::cout << "--" << std::endl;
		//std::cout << "before move: " << p.x << ", " << p.y << std::endl;
		// move particles
		//std::cout << "last node xy: " << p.last_node_x << ", " << p.last_node_y << std::endl;
		double current_robot_distance_from_last_node = p.get_distance_from_last_node(estimated_pose(0), estimated_pose(1));
		//std::cout << "robot_moved_distance: " << robot_moved_distance << std::endl;
		p.move(robot_moved_distance + rand(mt), map.edges[p.current_edge_index].direction);

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
			if(map.edges[p.current_edge_index].distance < p.get_particle_distance_from_last_node()){
				// arrived ???
				//std::cout << "particle arrived at node" << std::endl;
				p.near_node_flag = true;
				p.last_node_x = map.nodes[get_index_from_id(map.edges[p.current_edge_index].node1_id)].point.x;
				p.last_node_y = map.nodes[get_index_from_id(map.edges[p.current_edge_index].node1_id)].point.y;
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
				p.current_edge_index = get_next_edge_index_from_edge_index(p.current_edge_index);
				p.x = map.nodes[get_index_from_id(map.edges[p.current_edge_index].node0_id)].point.x; 
				p.y = map.nodes[get_index_from_id(map.edges[p.current_edge_index].node0_id)].point.y;
				// particle's moved distance is more suitable ??
				p.move(particle_distance_from_last_node + rand(mt), map.edges[p.current_edge_index].direction);
				//std::cout << map.edges[p.current_edge_index] << std::endl;
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

	for(auto p : particles){
		if(unique_edge_flag && ((p.current_edge_index != particles[0].current_edge_index) || p.near_node_flag)){
			// not unique if at least one particle on an edge different from p[0].
			unique_edge_flag = false;	
		}
	}

	if(unique_edge_flag){
		// inappropriate
		unique_edge_index = particles[0].current_edge_index;
	}

	// resampling
	if(resampling_count % 3 == 0){
		resampling();
		resampling_count = 0;
	}
	resampling_count++;
	std::cout << "unique edge index: " << unique_edge_index << std::endl;
	std::cout << "unique edge flag: " << unique_edge_flag << std::endl;
	
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

int NodeEdgeLocalizer::get_next_edge_index_from_edge_index(int index)
{
	double min_direction_diff = 100;
	int min_index = -1;
	int size = map.edges.size();
	for(int i=0;i<size;i++){
		if(index != i){
			if(map.edges[index].node1_id == map.edges[i].node0_id){
				// double diff = map.edges[i].direction - map.edges[index].direction;
				double diff = map.edges[i].direction - estimated_yaw;
				diff = fabs(pi_2_pi(diff));
				if(min_direction_diff > diff){
					min_direction_diff = diff;
					min_index = i;
				}
			}
		}
	}
	return min_index;
}

void NodeEdgeLocalizer::manage_passed_edge(int edge_index)
{
	if(edge_index != last_line_edge_index){
		// entered new edge
		std::cout << "!!! new unique edge !!!" << std::endl;
		std::cout << "index: " << edge_index << std::endl;
		double angle_diff = fabs(pi_2_pi(map.edges[edge_index].direction - map.edges[last_line_edge_index].direction));
		if(angle_diff > CONTINUOUS_LINE_THRESHOLD){
			end_line_edge_index = last_line_edge_index;
			std::cout << begin_line_edge_index << ", " << end_line_edge_index << ", " << last_line_edge_index << std::endl;
			int begin_node_index = get_index_from_id(map.edges[begin_line_edge_index].node0_id);
			int end_node_index = get_index_from_id(map.edges[end_line_edge_index].node1_id);
			amsl_navigation_msgs::Node begin_node = map.nodes[begin_node_index];
			amsl_navigation_msgs::Node end_node = map.nodes[end_node_index];
			double distance = sqrt(square(begin_node.point.x - end_node.point.x) + square(begin_node.point.y - end_node.point.y));
			if(distance > MIN_LINE_LENGTH){
				std::cout << "begin_node: \n" << begin_node << std::endl;;
				std::cout << "end_node: \n" << end_node << std::endl;;
				double line_angle = atan2((end_node.point.y - begin_node.point.y), (end_node.point.x - begin_node.point.x));
				std::cout << "line angle added !!!: " << line_angle << std::endl;
				passed_line_directions.push_back(line_angle);
				Eigen::Vector3d node_point;
				node_point << end_node.point.x, end_node.point.y, 0.0;
				passed_nodes.push_back(node_point);
				begin_line_edge_index = edge_index;
				int i=0;
				for(auto pn : passed_nodes){
					std::cout << "passed_node[" << i << "]: " << pn << std::endl;
					i++;
				}
			}
		}else{
			// extension of a straight line
			end_line_edge_index = last_line_edge_index; 
		}
		last_line_edge_index = edge_index;
		std::cout << "line count: " << passed_line_directions.size() << std::endl;
	}
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
	passed_nodes.clear();
	passed_line_directions.clear();
	linear_trajectories.clear();
	begin_line_edge_index = unique_edge_index;
	end_line_edge_index = unique_edge_index;
	last_line_edge_index = unique_edge_index;
	clear_flag = false;
}

void NodeEdgeLocalizer::visualize_lines(void)
{
	visualization_msgs::Marker lines_marker;
	lines_marker.header.frame_id = map.header.frame_id;
	lines_marker.header.stamp = ros::Time::now();
	lines_marker.ns = "line_viz";
	lines_marker.action = visualization_msgs::Marker::ADD;
	lines_marker.pose.orientation.w = 1;
	lines_marker.id = 0;
	lines_marker.type = visualization_msgs::Marker::LINE_LIST;
	lines_marker.scale.x = 0.1;
	lines_marker.color.r = 1.0;
	lines_marker.color.g = 0.0;
	lines_marker.color.b = 1.0;
	lines_marker.color.a = 1.0;
	for(auto traj : linear_trajectories){
		geometry_msgs::Point p;
		p.x = (*traj.begin())(0);
		p.y = (*traj.begin())(1);
		p.z = (*traj.begin())(2);
		lines_marker.points.push_back(p);
		p.x = (*(traj.end() - 1))(0);
		p.y = (*(traj.end() - 1))(1);
		p.z = (*(traj.end() - 1))(2);
		lines_marker.points.push_back(p);
	}
	lines_pub.publish(lines_marker);
}

void NodeEdgeLocalizer::remove_curve_from_trajectory(std::vector<Eigen::Vector3d>& traj)
{
	for(int i=0;i<POSE_NUM_PCA;i++){
		if(get_length_of_trajectory(traj) < MIN_LINE_LENGTH){
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
		estimated_edge = map.edges[unique_edge_index];
		last_node = map.nodes[get_index_from_id(estimated_edge.node0_id)];
		last_node_point << last_node.point.x, last_node.point.y, 0.0;
	}
	double distance_from_last_node = (estimated_pose - last_node_point).norm();
	estimated_edge.progress = distance_from_last_node / estimated_edge.distance; 
	edge_pub.publish(estimated_edge);
}

double NodeEdgeLocalizer::get_slope_angle(double angle)
{
	if(angle > M_PI / 2.0){
		angle -= M_PI;
	}else if(angle < -M_PI / 2.0){
		angle += M_PI;
	}
	return angle;
}
