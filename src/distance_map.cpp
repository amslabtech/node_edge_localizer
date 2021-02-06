/**
 * @file distance_map.cpp
 * @author amsl
 */
#include "node_edge_localizer/distance_map.h"
#include <chrono>

#include "node_edge_localizer/kdtree.h"

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
    kdtree::KDTree kdtree;
    kdtree.set_data(map);
    const auto start = std::chrono::system_clock::now();
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
    std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start).count() << "[ms]" << std::endl;
    int count = 0;
    #pragma omp parallel for reduction(+:count)
    for(unsigned int  ix=0;ix<x_size_;ix++){
        for(unsigned int iy=0;iy<y_size_;iy++){
            const unsigned int grid_index = iy * x_size_ + ix;
            const double x = ix * resolution + min_x_ - margin_2_;
            const double y = iy * resolution + min_y_ - margin_2_;
            double min_distance = 1e6;
            unsigned int min_index = 0;
            const int nearest_node_id = map.nodes[kdtree.find_nearest(Eigen::Vector2d(x, y))].id;
            if(nearest_node_id < 0){
                ROS_INFO_STREAM("nearest node of " << x << ", " << y << " not found");
            }
            for(unsigned int j=0;j<EDGE_NUM;j++){
                if((map.edges[j].node0_id != nearest_node_id) && (map.edges[j].node1_id != nearest_node_id)){
                    continue;
                }
                const double d = get_distance_from_edge(map, map.edges[j], x, y);
                if(d < min_distance){
                    min_distance = d;
                    min_index = j;
                }
                ++count;
            }
            const EdgeIndexWithDistance ei = {min_index, min_distance};
            map_[grid_index] = ei;
        }
    }
    edge_num_ = map.edges.size();
    std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start).count() << "[ms]" << std::endl;
    std::cout << "edge num: " << EDGE_NUM << std::endl;
    std::cout << "x_size: " << x_size_ << std::endl;
    std::cout << "y_size: " << y_size_ << std::endl;
    std::cout << "loop num: " << EDGE_NUM * x_size_ * y_size_ << std::endl;
    std::cout << "loop count: " << count << std::endl;
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
