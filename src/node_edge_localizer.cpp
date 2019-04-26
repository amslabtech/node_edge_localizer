#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include "Eigen/Dense"
#include "Eigen/Geometry"

#include "amsl_navigation_msgs/Node.h"
#include "amsl_navigation_msgs/Edge.h"
#include "amsl_navigation_msgs/NodeEdgeMap.h"

class NodeEdgeLocalizer
{
public:
	NodeEdgeLocalizer(void);

	void map_callback(const amsl_navigation_msgs::NodeEdgeMapConstPtr&);
	void odom_callback(const nav_msgs::OdometryConstPtr&);
	void process(void);
	void get_node_from_id(int, amsl_navigation_msgs::Node&);
	double pi_2_pi(double);
	void initialize(void);
	double get_curvature_from_trajectory(std::vector<Eigen::Vector3d>&);
	double get_angle_from_trajectory(std::vector<Eigen::Vector3d>&);
	void get_slope_from_trajectory(std::vector<Eigen::Vector3d>&, Eigen::Vector2d&);
	void calculate_pca(std::vector<Eigen::Vector3d>&, Eigen::Vector2d&, Eigen::Matrix2d&);
	void calculate_affine_tranformation(Eigen::Affine3d&);
	void calculate_affine_tranformation_tentatively(Eigen::Affine3d&);
	void get_intersection_from_trajectories(std::vector<Eigen::Vector3d>&, std::vector<Eigen::Vector3d>&, Eigen::Vector3d&);
	double get_angle_from_lines(Eigen::Vector3d&, Eigen::Vector3d&, Eigen::Vector3d&, Eigen::Vector3d&);
	double get_distance_from_trajectory(std::vector<Eigen::Vector3d>&);
	double square(double);
	int get_index_from_id(int);
	double calculate_trajectory_curvature(void);

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

	ros::NodeHandle nh;
	ros::NodeHandle private_nh;

	ros::Publisher pose_pub;
	ros::Publisher edge_pub;
	ros::Subscriber map_sub;
	ros::Subscriber odom_sub;

	tf::TransformListener listener;
	tf::StampedTransform transform;

	amsl_navigation_msgs::NodeEdgeMap map;
	amsl_navigation_msgs::Edge estimated_edge;
	bool map_subscribed;
	Eigen::Vector3d estimated_pose;
	double estimated_yaw;
	bool init_flag;
	// estimated edge(line) from odom 
	std::vector<Eigen::Vector3d> trajectory;
	// estimated edges from odom 
	std::vector<std::vector<Eigen::Vector3d> > trajectories;
	// correct odom to edge
	Eigen::Affine3d odom_correction;
	double yaw_correction;
	double yaw;
	double last_yaw;
	bool first_edge_flag;
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

	private_nh.param("HZ", HZ, {50});
	private_nh.param("INIT_NODE0_ID", INIT_NODE0_ID, {0});
	private_nh.param("INIT_NODE1_ID", INIT_NODE1_ID, {1});
	private_nh.param("INIT_PROGRESS", INIT_PROGRESS, {0.0});
	private_nh.param("INIT_YAW", INIT_YAW, {0.0});
	private_nh.param("CURVATURE_THRESHOLD", CURVATURE_THRESHOLD, {0.01});
	private_nh.param("POSE_NUM_PCA", POSE_NUM_PCA, {37});
	private_nh.param("MIN_LINE_SIZE", MIN_LINE_SIZE, {80});
	private_nh.param("MIN_LINE_LENGTH", MIN_LINE_LENGTH, {8.6});

	map_subscribed = false;
	init_flag = true;
	yaw = 0.0;
	last_yaw = 0.0;
	first_edge_flag = true;

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
}

void NodeEdgeLocalizer::map_callback(const amsl_navigation_msgs::NodeEdgeMapConstPtr& msg)
{
	map = *msg;
	map_subscribed = true;
}

void NodeEdgeLocalizer::odom_callback(const nav_msgs::OdometryConstPtr& msg)
{
	Eigen::Vector3d odom_pose;
	odom_pose << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
	odom_pose << odom_pose(0) * cos(INIT_YAW) - odom_pose(1) * sin(INIT_YAW) + map.nodes[get_index_from_id(INIT_NODE0_ID)].point.x,
		         odom_pose(0) * sin(INIT_YAW) + odom_pose(1) * cos(INIT_YAW) + map.nodes[get_index_from_id(INIT_NODE0_ID)].point.y;

	estimated_pose = odom_correction * odom_pose;
	estimated_yaw = tf::getYaw(msg->pose.pose.orientation) + yaw_correction + INIT_YAW;
	estimated_yaw = pi_2_pi(estimated_yaw);
}

void NodeEdgeLocalizer::process(void)
{
	ros::Rate loop_rate(HZ);
	
	while(ros::ok()){
		if(map_subscribed){
			if(init_flag){
				initialize();
			}
			if(calculate_trajectory_curvature() > CURVATURE_THRESHOLD){
				if(trajectory.size() > MIN_LINE_SIZE){
					static Eigen::Vector2d last_slope;
					if(!first_edge_flag){
						Eigen::Vector2d slope;
						// get slope of current trajectory
						get_slope_from_trajectory(trajectory, slope);
						double diff_angle = acos(slope.dot(last_slope));
						if(diff_angle > M_PI / 2.0){
							diff_angle = M_PI - diff_angle;
						}
						if(diff_angle > M_PI / 5.5){
							// maybe different line
							if(get_distance_from_trajectory(trajectory) > MIN_LINE_LENGTH / 2.0){
								if(diff_angle > M_PI / 3.5 || get_distance_from_trajectory(trajectory) > MIN_LINE_LENGTH){
									// robot was turned
									trajectories.push_back(trajectory);
									last_slope = slope;
								}else{
									// NOT different line
									std::copy(trajectory.begin(), trajectory.end(), std::back_inserter(trajectories.back()));
									get_slope_from_trajectory(trajectories.back(), last_slope);
								}
							}
						}else{
							// same line 
							std::copy(trajectory.begin(), trajectory.end(), std::back_inserter(trajectories.back()));
							get_slope_from_trajectory(trajectories.back(), last_slope);
							last_yaw = yaw;
						}
					}else{
						// first edge
						if(get_distance_from_trajectory(trajectory) > MIN_LINE_LENGTH){
							get_slope_from_trajectory(trajectory, last_slope);
							trajectories.push_back(trajectory);
							last_yaw = yaw;
							first_edge_flag = false;
						}
					}
				}else{
					// maybe robot is turning
				}
				trajectory.clear();
			}else{
				trajectory.push_back(estimated_pose);
			}
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
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

double NodeEdgeLocalizer::pi_2_pi(double angle)
{
	return atan2(sin(angle), cos(angle));
}

void NodeEdgeLocalizer::initialize(void)
{
	estimated_edge.node0_id = INIT_NODE0_ID;	
	estimated_edge.node1_id = INIT_NODE1_ID;	
	estimated_edge.progress = INIT_PROGRESS;
	amsl_navigation_msgs::Node node0;
	get_node_from_id(estimated_edge.node0_id, node0);
	amsl_navigation_msgs::Node node1;
	get_node_from_id(estimated_edge.node1_id, node1);
	estimated_edge.distance = (node0.point.x - node1.point.x) * (node0.point.x - node1.point.x) + (node0.point.y - node1.point.y) * (node0.point.y - node1.point.y); 
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

void NodeEdgeLocalizer::calculate_affine_tranformation(Eigen::Affine3d& affine_transformation)
{
	// It represents B(i) in paper
	Eigen::Vector3d intersection_point_i;
	//get_intersection_from_trajectories(trajectories, intersection_point_i, 0);
	// It represents B(i-1) in paper
	Eigen::Vector3d intersection_point_i_1;
	//get_intersection_from_trajectories(trajectories, intersection_point_i_1, 1);
	// It represents N(i) in paper
	Eigen::Vector3d map_node_point_i;
	// It represents N(i-1) in paper
	Eigen::Vector3d map_node_point_i_1;

	double theta = get_angle_from_lines(intersection_point_i, intersection_point_i_1, map_node_point_i, map_node_point_i_1);

	Eigen::Translation<double, 3> t1(intersection_point_i - map_node_point_i);
	Eigen::Translation<double, 3> t2(-intersection_point_i);
	Eigen::Matrix3d rotation;
	rotation = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ());
	affine_transformation = t1 * rotation * t2;
}

void NodeEdgeLocalizer::calculate_affine_tranformation_tentatively(Eigen::Affine3d& affine_transformation)
{
	// It represents B(i) in paper
	Eigen::Vector3d intersection_point_i;
	//get_intersection_from_trajectories(trajectories, intersection_point_i);
	// It represents B(i-1) in paper
	Eigen::Vector3d intersection_point_i_1;
	//get_intersection_from_trajectories(trajectories, intersection_point_i_1);
	// It represents N(i) in paper
	Eigen::Vector3d map_node_point_i;
	// It represents N(i-1) in paper
	Eigen::Vector3d map_node_point_i_1;

	double theta = get_angle_from_lines(intersection_point_i, intersection_point_i_1, map_node_point_i, map_node_point_i_1);

	Eigen::Translation<double, 3> t1(intersection_point_i - map_node_point_i);
	Eigen::Translation<double, 3> t2(-intersection_point_i);
	Eigen::Matrix3d rotation;
	rotation = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ());
	affine_transformation = t1 * rotation * t2;
}

void NodeEdgeLocalizer::get_intersection_from_trajectories(std::vector<Eigen::Vector3d>& trajectory_0, std::vector<Eigen::Vector3d>& trajectory_1, Eigen::Vector3d& intersection_point)
{
	Eigen::Vector2d center_0, center_1, slope_0, slope_1;
	double x1y2, x2y1, det;

	get_slope_from_trajectory(trajectory_0, slope_0);
	get_slope_from_trajectory(trajectory_1, slope_1);

	x1y2 = slope_0(0) * slope_1(1);
	x2y1 = slope_1(0) * slope_0(1);

	det = x1y2 - x2y1;

	if(fabs(det) < 1e-5){
		intersection_point << 0, 0, 0;
	}else{
		intersection_point << -(x2y1 * center_0(0) - slope_0(0) * slope_1(0) * (center_0(1) - center_1(1) - x1y2 * center_1(0))) / det,
		                       (x1y2 * center_0(1) - slope_0(1) * slope_1(1) * (center_0(0) - center_1(0) - x2y1 * center_1(1))) / det,
		                      0.0;
	}
}

double NodeEdgeLocalizer::get_angle_from_lines(Eigen::Vector3d& line0_p0, Eigen::Vector3d& line0_p1, Eigen::Vector3d& line1_p0, Eigen::Vector3d& line1_p1)
{
	double v0_x = line0_p0(0) - line0_p1(0);
	double v0_y = line0_p0(1) - line0_p1(1);
	double v1_x = line1_p0(0) - line1_p1(0);
	double v1_y = line1_p0(1) - line1_p1(1);
	return acos((v0_x * v1_x + v0_y * v1_y) / (sqrt(v0_x * v0_x + v0_y * v0_y) * sqrt(v1_x * v1_x + v1_y * v1_y)));
}

double NodeEdgeLocalizer::get_distance_from_trajectory(std::vector<Eigen::Vector3d>& traj)
{
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
	static std::vector<Eigen::Vector3d> pose_buffer;
	pose_buffer.resize(POSE_NUM_PCA);

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

	Eigen::EigenSolver<Eigen::Matrix2d> es(covariance_matrix);
	Eigen::Vector2d values = es.eigenvalues().real();

	double curvature = 100;
	if(values(0) + values(1)){
		curvature = (values(0) < values(1) ? values(0) : values(1)) / (values(0) + values(1));
	}
	count = (count + 1) % POSE_NUM_PCA;

	return curvature;
}
