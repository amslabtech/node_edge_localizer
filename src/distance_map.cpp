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
{

}

void DistanceMap::make_distance_map(const amsl_navigation_msgs::NodeEdgeMap& map, double resolution)
{
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
    map_.resize(x_size_ * y_size_);
    const unsigned int EDGE_NUM = map.edges.size();
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
}

double DistanceMap::get_distance_from_edge(const amsl_navigation_msgs::NodeEdgeMap& m, const amsl_navigation_msgs::Edge& e, double x, double y)
{
    //TODO: line segment version
    amsl_navigation_msgs::Node n0, n1;
    for(const auto& n : m.nodes){
        if(n.id == e.node0_id){
            n0 = n;
        }
        if(n.id == e.node1_id){
            n1 = n;
        }
    }
    const double a = (n1.point.y - n0.point.y) / (n1.point.x - n0.point.x);
    const double b = -1;
    const double c = n0.point.y - a * n0.point.x;
    return abs(a * x + b * y + c) / sqrt(a * a + b * b);
}

double DistanceMap::get_min_distance_from_edge(double x, double y)
{
    const unsigned int ix = (x - min_x_ + margin_2_) * grid_per_meter_;
    const unsigned int iy = (y - min_y_ + margin_2_) * grid_per_meter_;
    const unsigned int index = iy * x_size_ + ix;
    return map_[index].distance_;
}

unsigned int DistanceMap::get_nearest_edge_index(double x, double y)
{
    const unsigned int ix = (x - min_x_ + margin_2_) * grid_per_meter_;
    const unsigned int iy = (y - min_y_ + margin_2_) * grid_per_meter_;
    const unsigned int index = iy * x_size_ + ix;
    return map_[index].nearest_edge_index_;
}

std::tuple<std::vector<EdgeIndexWithDistance>, double, double, double, double> DistanceMap::get_data(void) const
{
    return std::forward_as_tuple(map_, min_x_ - margin_2_, max_x_ + margin_2_, min_y_ - margin_2_, max_y_ + margin_2_);
}
}