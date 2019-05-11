#include "node_edge_localizer/node_edge_map_management.h"

NodeEdgeMapManagement::NodeEdgeMapManagement(void)
{

}

void NodeEdgeMapManagement::set_parameters(double continuous_line_threshold, double min_line_length)
{
	CONTINUOUS_LINE_THRESHOLD = continuous_line_threshold;
	MIN_LINE_LENGTH = min_line_length;
}

void NodeEdgeMapManagement::set_map(amsl_navigation_msgs::NodeEdgeMap& _map)
{
	map = _map;	
}

void NodeEdgeMapManagement::get_node_from_id(int id, amsl_navigation_msgs::Node& node)
{
	for(auto n : map.nodes){
		if(n.id == id){
			node = n;
			return;
		}
	}
}

void NodeEdgeMapManagement::get_edge_from_node_id(int node0_id, int node1_id, amsl_navigation_msgs::Edge& edge)
{
	for(auto e : map.edges){
		if(e.node0_id == node0_id && e.node1_id == node1_id){
			edge = e;
			return;
		}
	}
}

int NodeEdgeMapManagement::get_edge_index_from_node_id(int node0_id, int node1_id)
{
	int index = 0;
	for(auto e : map.edges){
		if(e.node0_id == node0_id && e.node1_id == node1_id){
			return index;
		}
		index++;
	}
}

int NodeEdgeMapManagement::get_index_from_id(int id)
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

std::string NodeEdgeMapManagement::get_map_header_frame_id(void)
{
	return map.header.frame_id;
}

int NodeEdgeMapManagement::get_next_edge_index_from_edge_index(int index, double estimated_yaw)
{
	double min_direction_diff = 100;
	int min_index = -1;
	int size = map.edges.size();
	for(int i=0;i<size;i++){
		if(index != i){
			if(map.edges[index].node1_id == map.edges[i].node0_id){
				// double diff = map.edges[i].direction - map.edges[index].direction;
				double diff = map.edges[i].direction - estimated_yaw;
				diff = fabs(Calculation::pi_2_pi(diff));
				if(min_direction_diff > diff){
					min_direction_diff = diff;
					min_index = i;
				}
			}
		}
	}
	return min_index;
}

void NodeEdgeMapManagement::manage_passed_edge(int edge_index)
{
	if(edge_index != last_line_edge_index){
		// entered new edge
		std::cout << "!!! new unique edge !!!" << std::endl;
		std::cout << "index: " << edge_index << std::endl;
		if(map.edges[last_line_edge_index].node1_id != map.edges[edge_index].node0_id){
			// not connected edges 
			int index = search_interpolating_edge(last_line_edge_index, edge_index);
			if(index >= 0){
				std::cout << "interpolatin with edge index: " << index << std::endl;
				edge_index = index;
			}
		}
		double angle_diff = fabs(Calculation::pi_2_pi(map.edges[edge_index].direction - map.edges[last_line_edge_index].direction));
		if(angle_diff > CONTINUOUS_LINE_THRESHOLD){
			end_line_edge_index = last_line_edge_index;
			std::cout << begin_line_edge_index << ", " << end_line_edge_index << ", " << last_line_edge_index << std::endl;
			int begin_node_index = get_index_from_id(map.edges[begin_line_edge_index].node0_id);
			int end_node_index = get_index_from_id(map.edges[end_line_edge_index].node1_id);
			amsl_navigation_msgs::Node begin_node = map.nodes[begin_node_index];
			amsl_navigation_msgs::Node end_node = map.nodes[end_node_index];
			double distance = sqrt(Calculation::square(begin_node.point.x - end_node.point.x) + Calculation::square(begin_node.point.y - end_node.point.y));
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
					std::cout << "passed_node[" << i << "]: \n" << pn << std::endl;
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

int NodeEdgeMapManagement::search_interpolating_edge(int edge0_index, int edge1_index)
{
	int edge0_node1_id = map.edges[edge0_index].node1_id;
	int edge1_node0_id = map.edges[edge1_index].node0_id;
	int index = 0;
	for(auto e : map.edges){
		if(e.node0_id == edge0_node1_id && e.node1_id == edge1_node0_id){
			return index;
		}
		index++;
	}
	return -1;
}
