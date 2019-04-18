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

private:
	double HZ;
	int INIT_NODE0_ID;
	int INIT_NODE1_ID;
	double INIT_PROGRESS;
	double INIT_YAW;

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
	Eigen::Vector3d last_pose;
	Eigen::Vector3d current_pose;
	bool init_flag;
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

	map_subscribed = false;
	init_flag = true;

	std::cout << "=== node_edge_localizer ===" << std::endl;
	std::cout << "HZ: " << HZ << std::endl;
	std::cout << "INIT_NODE0_ID: " << INIT_NODE0_ID << std::endl;
	std::cout << "INIT_NODE1_ID: " << INIT_NODE1_ID << std::endl;
	std::cout << "INIT_PROGRESS: " << INIT_PROGRESS << std::endl;
	std::cout << "INIT_YAW: " << INIT_YAW << std::endl;
}

void NodeEdgeLocalizer::map_callback(const amsl_navigation_msgs::NodeEdgeMapConstPtr& msg)
{
	map = *msg;
	map_subscribed = true;
}

void NodeEdgeLocalizer::odom_callback(const nav_msgs::OdometryConstPtr& msg)
{
}

void NodeEdgeLocalizer::process(void)
{
	ros::Rate loop_rate(HZ);
	while(ros::ok()){
		if(map_subscribed){
			if(init_flag){
				initialize();
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
