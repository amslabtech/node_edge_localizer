#ifndef __NODE_EDGE_MAP_MANAGEMENT
#define __NODE_EDGE_MAP_MANAGEMENT

#include <ros/ros.h>

#include <Eigen/Dense>

#include "amsl_navigation_msgs/Node.h"
#include "amsl_navigation_msgs/Edge.h"
#include "amsl_navigation_msgs/NodeEdgeMap.h"

#include "node_edge_localizer/calculation.h"

class NodeEdgeMapManagement
{
public:
	NodeEdgeMapManagement(void);

	void set_parameters(double, double);
	void set_map(amsl_navigation_msgs::NodeEdgeMap&);
	void get_node_from_id(int id, amsl_navigation_msgs::Node&);
	void get_edge_from_node_id(int, int, amsl_navigation_msgs::Edge&);
	int get_edge_index_from_node_id(int, int);
	int get_index_from_id(int);
	std::string get_map_header_frame_id(void);
	int get_next_edge_index_from_edge_index(int, double);
	void manage_passed_edge(int);
	int search_interpolating_edge(int, int);

private:
	double CONTINUOUS_LINE_THRESHOLD;
	double MIN_LINE_LENGTH;

	amsl_navigation_msgs::NodeEdgeMap map;
	std::vector<Eigen::Vector3d> passed_nodes;
	std::vector<double> passed_line_directions;
	int begin_line_edge_index;
	int end_line_edge_index;
	int last_line_edge_index;
};

#endif// __NODE_EDGE_MAP_MANAGEMENT
