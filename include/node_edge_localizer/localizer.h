/**
 * @file localizer.h
 * @author amsl
 */
#ifndef __NODE_EDGE_LOCALIZER_LOCALIZER_H
#define __NODE_EDGE_LOCALIZER_LOCALIZER_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>

namespace node_edge_localizer
{
class Localizer
{
public:
    Localizer(void);

    void process(void);
protected:
    ros::NodeHandle nh_;
    ros::NodeHandle local_nh_;
};
}// namespace node_edge_localizer

#endif// __NODE_EDGE_LOCALIZER_LOCALZER_H