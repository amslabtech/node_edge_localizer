/**
 * @file distance_map.h
 * @author amsl
 */
#ifndef __NODE_EDGE_LOCALIZER_DISTANCE_MAP_H
#define __NODE_EDGE_LOCALIZER_DISTANCE_MAP_H

#include <omp.h>

#include "amsl_navigation_msgs/NodeEdgeMap.h"

namespace node_edge_localizer
{
struct EdgeIndexWithDistance
{
    unsigned int nearest_edge_index_;
    double distance_;
};

class DistanceMap
{
public:
    DistanceMap(void);
    void make_distance_map(const amsl_navigation_msgs::NodeEdgeMap& map, double resolution);
    double get_distance_from_edge(const amsl_navigation_msgs::NodeEdgeMap& m, const amsl_navigation_msgs::Edge& e, double x, double y)
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
    /**
     * @brief Get distance from nearest edge
     * @param[in] x x position in (estimated) map frame
     * @param[in] y y position in (estimated) map frame
     */
    double get_min_distance_from_edge(double x, double y);
    unsigned int get_nearest_edge_index(double x, double y);
    std::tuple<std::vector<EdgeIndexWithDistance>, double, double, double, double> get_data(void) const;
    double get_squared_distance(double x0, double y0, double x1, double y1);
protected:
    //! margin from min/max position of map [m]
    double margin_;
    //! half of margin_
    double margin_2_;
    unsigned int x_size_;
    unsigned int y_size_;
    double min_x_;
    double max_x_;
    double min_y_;
    double max_y_;
    double resolution_;
    //! reciprocal of resolution_
    double grid_per_meter_;
    std::vector<EdgeIndexWithDistance> map_;
    unsigned int map_size_;
    unsigned int edge_num_;

    std::map<int, amsl_navigation_msgs::Node> id_to_node_;
};
}
#endif// __NODE_EDGE_LOCALIZER_DISTANCE_MAP_H
