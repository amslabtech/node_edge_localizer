/**
 * @file distance_map.h 
 * @author amsl
 */
#ifndef __NODE_EDGE_LOCALIZER_DISTANCE_MAP_H
#define __NODE_EDGE_LOCALIZER_DISTANCE_MAP_H

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
    std::tuple<std::vector<EdgeIndexWithDistance>, double, double, double, double> get_data(void) const;
protected:
    //! margin from min/max position of map [m]
    double margin_;
    unsigned int x_size_;
    unsigned int y_size_;
    double min_x_;
    double max_x_;
    double min_y_;
    double max_y_;
    double resolution_;
    std::vector<EdgeIndexWithDistance> map_;
};
}
#endif// __NODE_EDGE_LOCALIZER_DISTANCE_MAP_H