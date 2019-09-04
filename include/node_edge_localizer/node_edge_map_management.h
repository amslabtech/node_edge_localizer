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

    void set_parameters(int, int, double, double);
    void set_map(amsl_navigation_msgs::NodeEdgeMap&);
    void get_node_from_id(int id, amsl_navigation_msgs::Node&);
    void get_edge_from_node_id(int, int, amsl_navigation_msgs::Edge&);
    int get_node_index_from_id(int);
    int get_edge_index_from_node_id(int, int);
    std::string get_map_header_frame_id(void);
    int get_next_edge_index_from_edge_index(int, int, double);
    void manage_passed_edge(int);
    amsl_navigation_msgs::Edge get_edge_from_index(int);
    void get_candidate_edges(double, int, std::vector<amsl_navigation_msgs::Edge>&);
    int get_passed_line_directions_size(void);
    double get_passed_line_direction(int);
    void clear(int);
    Eigen::Vector3d get_passed_node(int);
    void show_line_edge_ids(void);
    void show_edge_from_index(int);
    double get_end_of_line_edge_distance(void);
    void get_begin_node_of_begin_line_edge(amsl_navigation_msgs::Node&);
    void get_end_node_of_last_line_edge(amsl_navigation_msgs::Node&);
    int get_edge_num(void);
	void get_edge_from_estimated_pose(double estimated_x, double estimated_y, double estimated_yaw, amsl_navigation_msgs::Edge& edge);

private:
    int search_interpolating_edge(int, int);

    int INIT_NODE0_ID;
    int INIT_NODE1_ID;
    double CONTINUOUS_LINE_THRESHOLD;
    double MIN_LINE_LENGTH;

    amsl_navigation_msgs::NodeEdgeMap map;
    std::vector<Eigen::Vector3d> passed_nodes;
    std::vector<double> passed_line_directions;
    // for correction
    int begin_line_edge_index;
    int end_line_edge_index;
    int last_line_edge_index;
};

#endif// __NODE_EDGE_MAP_MANAGEMENT
