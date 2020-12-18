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
    double get_distance_from_edge(const amsl_navigation_msgs::NodeEdgeMap& map, const amsl_navigation_msgs::Edge& e, double x, double y);
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