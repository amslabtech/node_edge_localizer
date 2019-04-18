#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>

#include "amsl_navigation_msgs/Node.h"
#include "amsl_navigation_msgs/Edge.h"
#include "amsl_navigation_msgs/NodeEdgeMap.h"

class NodeEdgeLocalizer
{
public:
	NodeEdgeLocalizer(void);

	void map_callback(const amsl_navigation_msgs::NodeEdgeMapConstPtr&);
	void process(void);
	void get_node_from_id(int, amsl_navigation_msgs::Node&);
	double pi_2_pi(double);

private:
	double HZ;

	ros::NodeHandle nh;
	ros::NodeHandle private_nh;

	ros::Publisher pose_pub;
	ros::Publisher edge_pub;
	ros::Subscriber map_sub;

	tf::TransformListener listener;
	tf::StampedTransform transform;

	amsl_navigation_msgs::NodeEdgeMap map;
	bool map_subscribed;
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
	pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/estimated_pose/pose", 1);
	edge_pub = nh.advertise<amsl_navigation_msgs::Edge>("/estimated_pose/edge", 1);

	private_nh.param("HZ", HZ, {50});

	map_subscribed = false;

	std::cout << "=== node_edge_localizer ===" << std::endl;
	std::cout << "HZ: " << HZ << std::endl;
}

void NodeEdgeLocalizer::map_callback(const amsl_navigation_msgs::NodeEdgeMapConstPtr& msg)
{
	map = *msg;
	map_subscribed = true;
}

void NodeEdgeLocalizer::process(void)
{
	ros::Rate loop_rate(HZ);
	while(ros::ok()){
		if(map_subscribed){
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
