#include "node_edge_localizer/node_edge_map_management.h"

NodeEdgeMapManagement::NodeEdgeMapManagement(void)
{

}

void NodeEdgeMapManagement::set_parameters(int init_node0_id, int init_node1_id, double continuous_line_threshold, double min_line_length)
{
    INIT_NODE0_ID = init_node0_id;
    INIT_NODE1_ID = init_node1_id;
    CONTINUOUS_LINE_THRESHOLD = continuous_line_threshold;
    MIN_LINE_LENGTH = min_line_length;
    begin_line_edge_index = get_edge_index_from_node_id(INIT_NODE0_ID, INIT_NODE1_ID);
    end_line_edge_index = get_edge_index_from_node_id(INIT_NODE0_ID, INIT_NODE1_ID);
    last_line_edge_index = get_edge_index_from_node_id(INIT_NODE0_ID, INIT_NODE1_ID);
}

int NodeEdgeMapManagement::get_next_edge_index_from_edge_index(int index, int id, double estimated_yaw)
{
    double min_direction_diff = 100;
    int min_index = -1;
    int size = map.edges.size();
    if(id == get_edge_from_index(index).node1_id){
        // forward
        for(int i=0;i<size;i++){
            if(index != i){
                if(map.edges[index].node1_id == map.edges[i].node0_id){
                    double diff = map.edges[i].direction - estimated_yaw;
                    diff = fabs(Calculation::pi_2_pi(diff));
                    if(min_direction_diff > diff){
                        min_direction_diff = diff;
                        min_index = i;
                    }
                }
            }
        }
    }else{
        // back
        for(int i=0;i<size;i++){
            if(index != i){
                if(map.edges[index].node0_id == map.edges[i].node1_id){
                    double diff = map.edges[i].direction - estimated_yaw;
                    diff = fabs(Calculation::pi_2_pi(diff));
                    if(min_direction_diff > diff){
                        min_direction_diff = diff;
                        min_index = i;
                    }
                }
            }
        }
    }
    return min_index;
}

void NodeEdgeMapManagement::manage_passed_edge(int edge_index)
{
    static std::vector<int> line_edge_indices;
    if(edge_index != last_line_edge_index){
        // entered new edge
        std::cout << "!!! new unique edge !!!" << std::endl;
        std::cout << "index: " << edge_index << std::endl;
        amsl_navigation_msgs::Edge unique_edge = get_edge_from_index(edge_index);
        std::cout << unique_edge << std::endl;
        amsl_navigation_msgs::Edge last_unique_edge = get_edge_from_index(last_line_edge_index);
        if(last_unique_edge.node1_id == unique_edge.node0_id){
            std::cout << "\033[48;5;2m" << "entered next edge" << "\033[0m" <<  std::endl;
        }else if(map.edges[last_line_edge_index].node1_id != map.edges[edge_index].node0_id){
            // not connected edges
            std::cout << "\033[48;5;1m";
            std::cout << "skipped edge" << std::endl;
            int interpolating_edge_index = search_interpolating_edge(last_line_edge_index, edge_index);
            if(interpolating_edge_index >= 0){
                std::cout << "interpolating with edge index: " << interpolating_edge_index << std::endl;
                show_edge_from_index(interpolating_edge_index);
                amsl_navigation_msgs::Edge interpolating_edge = get_edge_from_index(interpolating_edge_index);
                if(last_unique_edge.node0_id != interpolating_edge.node1_id){
                    edge_index = interpolating_edge_index;
                    std::cout << "successfully interpolated" << std::endl;
                    show_line_edge_ids();
                }else{
                    std::cout << "!!! interpolation was interrupted! !!!" << std::endl;
                    if(begin_line_edge_index == last_line_edge_index){
                        std::cout << "begin_line_edge_index was updated to end_line_edge_index" << std::endl;
                        begin_line_edge_index = end_line_edge_index;
                        last_line_edge_index = end_line_edge_index;
                        show_line_edge_ids();
                    }else if(begin_line_edge_index == end_line_edge_index){
                        std::cout << "end_line_edge_index was updated to begin_line_edge_index" << std::endl;
                        end_line_edge_index = begin_line_edge_index;
                        last_line_edge_index = begin_line_edge_index;
                        show_line_edge_ids();
                    }else if(get_edge_from_index(end_line_edge_index).node1_id == interpolating_edge.node1_id){
                        std::cout << "last_line_edge_index was updated to end_line_edge_index" << std::endl;
                        last_line_edge_index = end_line_edge_index;
                        show_line_edge_ids();
                    }else{
                        std::cout << "!!! unknown error !!!" << std::endl;
                    }
                }
            }
            std::cout << "\033[0m" << std::endl;
        }else if(map.edges[last_line_edge_index].node0_id == map.edges[edge_index].node0_id){
            std::cout << "\033[48;5;4m";
            std::cout << "maybe last unique edge was invalid" << std::endl;
            std::cout << "\033[0m" << std::endl;
        }
        double angle_diff = fabs(Calculation::pi_2_pi(map.edges[edge_index].direction - map.edges[last_line_edge_index].direction));
        if(angle_diff > CONTINUOUS_LINE_THRESHOLD){
            std::cout << "edges was not line" << std::endl;
            end_line_edge_index = last_line_edge_index;
            show_line_edge_ids();
            int begin_node_index = get_node_index_from_id(map.edges[begin_line_edge_index].node0_id);
            int end_node_index = get_node_index_from_id(map.edges[end_line_edge_index].node1_id);
            amsl_navigation_msgs::Node begin_node = map.nodes[begin_node_index];
            amsl_navigation_msgs::Node end_node = map.nodes[end_node_index];
            double distance = sqrt(Calculation::square(begin_node.point.x - end_node.point.x) + Calculation::square(begin_node.point.y - end_node.point.y));
            if(distance > MIN_LINE_LENGTH){
                std::cout << "begin_node: \n" << begin_node << std::endl;;
                std::cout << "end_node: \n" << end_node << std::endl;;
                double line_angle = atan2((end_node.point.y - begin_node.point.y), (end_node.point.x - begin_node.point.x));
                Eigen::Vector3d node_point;
                node_point << end_node.point.x, end_node.point.y, 0.0;
                if(passed_nodes.size() > 0){
                    if((passed_nodes.back() - node_point).norm() > 1e-3){
                        passed_nodes.push_back(node_point);
                        std::cout << "passed nodes added: " << end_node.id << std::endl;
                        passed_line_directions.push_back(line_angle);
                        std::cout << "line angle added !!!: " << line_angle << std::endl;
                    }else{
                        std::cout << "passed node and line angle were not added due to duplication" << std::endl;
                    }
                }else{
                    passed_nodes.push_back(node_point);
                    std::cout << "passed nodes added: " << end_node.id << std::endl;
                    passed_line_directions.push_back(line_angle);
                    std::cout << "line angle added !!!: " << line_angle << std::endl;
                }
                begin_line_edge_index = edge_index;
                for(unsigned int i=0;i<passed_nodes.size();i++){
                    std::cout << "passed_node[" << i << "]: \n" << passed_nodes[i] << std::endl;
                    std::cout << "passed_line_directions[" << i << "]: \n" << passed_line_directions[i] << std::endl;
                }
            }
        }else{
            // extension of a straight line
            std::cout << "end line edge was updated" << std::endl;
            end_line_edge_index = last_line_edge_index;
            show_line_edge_ids();
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

void NodeEdgeMapManagement::get_candidate_edges(double estimated_yaw, int current_edge_index, std::vector<amsl_navigation_msgs::Edge>& candidate_edges)
{
    amsl_navigation_msgs::Edge current_edge = map.edges[current_edge_index];
    candidate_edges.push_back(current_edge);
    for(auto e : map.edges){
        if(e.node0_id == current_edge.node0_id){
            if(M_PI - fabs(Calculation::pi_2_pi(e.direction - estimated_yaw)) > CONTINUOUS_LINE_THRESHOLD){
                if( M_PI - fabs(Calculation::pi_2_pi(e.direction - current_edge.direction)) > CONTINUOUS_LINE_THRESHOLD){
                    candidate_edges.push_back(e);
                }
            }
        }
    }
}

int NodeEdgeMapManagement::get_passed_line_directions_size(void)
{
    return passed_line_directions.size();
}

double NodeEdgeMapManagement::get_passed_line_direction(int index)
{
    return passed_line_directions[index];
}

void NodeEdgeMapManagement::clear(int unique_edge_index)
{
    begin_line_edge_index = unique_edge_index;
    end_line_edge_index = unique_edge_index;
    last_line_edge_index = unique_edge_index;
    manage_passed_edge(unique_edge_index);
    passed_nodes.clear();
    passed_line_directions.clear();
}

Eigen::Vector3d NodeEdgeMapManagement::get_passed_node(int index)
{
    return passed_nodes[index];
}

void NodeEdgeMapManagement::show_line_edge_ids(void)
{
    std::cout << "begin: ";
    show_edge_from_index(begin_line_edge_index);
    std::cout << "end: ";
    show_edge_from_index(end_line_edge_index);
    std::cout << "last: ";
    show_edge_from_index(last_line_edge_index);
}

void NodeEdgeMapManagement::show_edge_from_index(int index)
{
    amsl_navigation_msgs::Edge e;
    e = get_edge_from_index(index);
    std::cout << e.node0_id << " -> " << e.node1_id << std::endl;
}

double NodeEdgeMapManagement::get_end_of_line_edge_distance(void)
{
    return map.edges[end_line_edge_index].distance;
}

void NodeEdgeMapManagement::get_begin_node_of_begin_line_edge(amsl_navigation_msgs::Node& node)
{
    get_node_from_id(get_edge_from_index(begin_line_edge_index).node0_id, node);
}

void NodeEdgeMapManagement::get_end_node_of_last_line_edge(amsl_navigation_msgs::Node& node)
{
    get_node_from_id(get_edge_from_index(last_line_edge_index).node1_id, node);
}

void NodeEdgeMapManagement::get_edge_from_estimated_pose(double estimated_x, double estimated_y, double estimated_yaw, amsl_navigation_msgs::Edge& edge)
{
    amsl_navigation_msgs::Edge nearest_edge;
    double distance = 1e6;

    for(const auto e : map.edges){

        amsl_navigation_msgs::Node node0;
        get_node_from_id(e.node0_id, node0);
        amsl_navigation_msgs::Node node1;
        get_node_from_id(e.node1_id, node1);
        double distance_from_estimated_pose_to_edge = 10e6;

        if(node1.point.x - node0.point.x != 0.0){
            // ax+by+c=0
            double a = (node1.point.y - node0.point.y) / (node1.point.x - node0.point.x);
            double b = -1;
            double c = node0.point.y - a * node0.point.x;
            double distance_from_point_to_line = fabs(a * estimated_x + b * estimated_y + c) / sqrt(Calculation::square(a) + Calculation::square(b));
            // check the position of particle
            double square_distance_from_point_to_line = Calculation::square(distance_from_point_to_line);
            double square_distance_from_point_to_node0 = Calculation::square(estimated_x - node0.point.x) + Calculation::square(estimated_y - node0.point.y);
            double square_distance_from_point_to_node1 = Calculation::square(estimated_x - node1.point.x) + Calculation::square(estimated_y - node1.point.y);
            double square_distance_from_node0_to_node1 = Calculation::square(node0.point.x - node1.point.x) + Calculation::square(node0.point.y - node1.point.y);
            if(sqrt(square_distance_from_point_to_node0 - square_distance_from_point_to_line) + sqrt(square_distance_from_point_to_node1 - square_distance_from_point_to_line) == sqrt(square_distance_from_node0_to_node1)){
                distance_from_estimated_pose_to_edge = distance_from_point_to_line;
            }else if(square_distance_from_point_to_node0 > square_distance_from_point_to_node1){
                distance_from_estimated_pose_to_edge = sqrt(square_distance_from_point_to_node1);
            }else if(square_distance_from_point_to_node0 < square_distance_from_point_to_node1){
                distance_from_estimated_pose_to_edge = sqrt(square_distance_from_point_to_node0);
            }
        }else{
            distance_from_estimated_pose_to_edge = fabs(estimated_x - node1.point.x);
        }

        if(distance_from_estimated_pose_to_edge < distance && fabs(Calculation::pi_2_pi(e.direction - estimated_yaw)) < M_PI / 2){
            nearest_edge = e;
            distance = distance_from_estimated_pose_to_edge;
        }
    }
    edge = nearest_edge;
}

int NodeEdgeMapManagement::get_begin_line_edge_index(void)
{
    return begin_line_edge_index;
}

int NodeEdgeMapManagement::get_last_line_edge_index(void)
{
    return last_line_edge_index;
}

int NodeEdgeMapManagement::get_end_line_edge_index(void)
{
    return end_line_edge_index;
}
