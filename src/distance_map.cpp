/**
 * @file distance_map.cpp
 * @author amsl
 */
#include "node_edge_localizer/distance_map.h"

namespace node_edge_localizer
{
DistanceMap::DistanceMap(void)
: margin_(20.0)
, margin_2_(margin_ / 2.0)
, x_size_(0)
, y_size_(0)
, min_x_(0)
, max_x_(0)
, min_y_(0)
, max_y_(0)
, resolution_(0.1)
, grid_per_meter_(1.0 / resolution_)
, map_size_(0)
, edge_num_(0)
{

}

void DistanceMap::make_distance_map(const amsl_navigation_msgs::NodeEdgeMap& map, double resolution)
{
    for(const auto& n : map.nodes){
        id_to_node_[n.id] = n;
    }
    resolution_ = resolution;
    grid_per_meter_ = 1.0 / resolution_;
    margin_2_ = margin_ / 2.0;
    if(map.nodes.size() == 0){
        std::cout << "error: map has no node!" << std::endl;
        return;
    }
    min_x_ = map.nodes[0].point.x;
    max_x_ = map.nodes[0].point.x;
    min_y_ = map.nodes[0].point.y;
    max_y_ = map.nodes[0].point.y;
    // search min/max position in map
    for(const auto& n : map.nodes){
        min_x_ = std::min(n.point.x, min_x_);
        max_x_ = std::max(n.point.x, max_x_);
        min_y_ = std::min(n.point.y, min_y_);
        max_y_ = std::max(n.point.y, max_y_);
    }
    x_size_ = (max_x_ - min_x_ + margin_) * grid_per_meter_;
    y_size_ = (max_y_ - min_y_ + margin_) * grid_per_meter_;
    map_size_ = x_size_ * y_size_;
    map_.resize(map_size_);
    const unsigned int EDGE_NUM = map.edges.size();
    #pragma omp parallel for
    for(unsigned int  ix=0;ix<x_size_;ix++){
        for(unsigned int iy=0;iy<y_size_;iy++){
            const unsigned int grid_index = iy * x_size_ + ix;
            const double x = ix * resolution + min_x_ - margin_2_;
            const double y = iy * resolution + min_y_ - margin_2_;
            double min_distance = 1e6;
            unsigned int min_index = 0;
            for(unsigned int j=0;j<EDGE_NUM;j++){
                const double d = get_distance_from_edge(map, map.edges[j], x, y);
                if(d < min_distance){
                    min_distance = d;
                    min_index = j;
                }
            }
            const EdgeIndexWithDistance ei = {min_index, min_distance};
            map_[grid_index] = ei;
        }
    }
    edge_num_ = map.edges.size();
}

double DistanceMap::get_distance_from_edge(const amsl_navigation_msgs::NodeEdgeMap& m, const amsl_navigation_msgs::Edge& e, double x, double y)
{
    amsl_navigation_msgs::Node n0, n1;
    n0 = id_to_node_[e.node0_id];
    n1 = id_to_node_[e.node1_id];
    // ref: https://boiledorange73.qrunch.io/entries/mir1mmohtOz9qkgq
    const double a = n1.point.x - n0.point.x;
    const double b = n1.point.y - n0.point.y;
    const double t = -(a * (n0.point.x - x) + b * (n0.point.y - y));
    const double a_b_squared_sum = a * a + b * b;
    if(t < 0){
        return sqrt(get_squared_distance(n0.point.x, n0.point.y, x, y));
    }else if(t > a_b_squared_sum){
        return sqrt(get_squared_distance(n1.point.x, n1.point.y, x, y));
    }else{
        // intersection is on the edge
        const double f = a * (n0.point.y - y) - b * (n0.point.x - x);
        return sqrt((f * f) / a_b_squared_sum);
    }
}

double DistanceMap::get_min_distance_from_edge(double x, double y)
{
    const unsigned int ix = (x - min_x_ + margin_2_) * grid_per_meter_;
    const unsigned int iy = (y - min_y_ + margin_2_) * grid_per_meter_;
    const unsigned int index = iy * x_size_ + ix;
    if(0 <= index && index < map_size_){
        return map_[index].distance_;
    }else{
        return -1;
    }
}

unsigned int DistanceMap::get_nearest_edge_index(double x, double y)
{
    const unsigned int ix = (x - min_x_ + margin_2_) * grid_per_meter_;
    const unsigned int iy = (y - min_y_ + margin_2_) * grid_per_meter_;
    const unsigned int index = iy * x_size_ + ix;
    if(0 <= index && index < map_size_){
        return map_[index].nearest_edge_index_;
    }else{
        return edge_num_;
    }
}

std::tuple<std::vector<EdgeIndexWithDistance>, double, double, double, double> DistanceMap::get_data(void) const
{
    return std::forward_as_tuple(map_, min_x_ - margin_2_, max_x_ + margin_2_, min_y_ - margin_2_, max_y_ + margin_2_);
}

double DistanceMap::get_squared_distance(double x0, double y0, double x1, double y1)
{
    const double x_1_0 = x1 - x0;
    const double y_1_0 = y1 - y0;
    const double d2 = x_1_0 * x_1_0 + y_1_0 * y_1_0;
    return d2;
}
}
