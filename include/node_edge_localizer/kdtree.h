#ifndef __NODE_EDGE_LOCALIZER_KDTREE_H
#define __NODE_EDGE_LOCALIZER_KDTREE_H

#include <memory>
#include <vector>

#include <ros/ros.h>

#include <Eigen/Dense>

#include "amsl_navigation_msgs/NodeEdgeMap.h"

namespace node_edge_localizer
{

namespace kdtree
{

struct Node
{
    int location;
    std::shared_ptr<Node> left_child;
    std::shared_ptr<Node> right_child;
    int axis;
};

class KDTree
{
public:
    KDTree(void)
    : k_(2)
    {
    }

    void set_data(const amsl_navigation_msgs::NodeEdgeMap& map)
    {
        points_.reserve(map.nodes.size());
        std::vector<int> indices;
        indices.reserve(map.nodes.size());
        for(auto it=map.nodes.begin();it!=map.nodes.end();++it){
            const Eigen::Vector2d p(it->point.x, it->point.y);
            points_.emplace_back(p);
            indices.emplace_back(std::distance(map.nodes.begin(), it));
        }
        root_ = make_kdtree(indices);
        // ROS_INFO_STREAM("root loc: " << root_->location);
    }

    std::shared_ptr<Node> make_kdtree(std::vector<int> indices, int depth = 0)
    {
        // ROS_INFO_STREAM("---");
        // ROS_INFO_STREAM("depth: " << depth);
        // ROS_INFO_STREAM("indices num: " << indices.size());
        if(indices.empty()){
            return nullptr;
        }
        const int axis = depth % k_;
        const int num = indices.size();
        const int mid = num / 2;
        // ROS_INFO_STREAM("axis: " << axis);
        // ROS_INFO_STREAM("num: " << num);
        // ROS_INFO_STREAM("mid: " << mid);

        auto compare = [&](int lhs, int rhs) -> bool
        {
            return points_[lhs][axis] < points_[rhs][axis];
        };
        std::nth_element(indices.begin(), indices.begin() + mid, indices.end(), compare);

        std::shared_ptr<Node> n(new Node);
        n->location = indices[mid];
        n->axis = axis;
        // ROS_INFO_STREAM("left");
        n->left_child = make_kdtree(std::vector<int>(indices.begin(), indices.begin() + mid), depth + 1);
        // ROS_INFO_STREAM("right");
        n->right_child = make_kdtree(std::vector<int>(indices.begin() + mid + 1, indices.end()), depth + 1);
        // ROS_INFO_STREAM("return");
        return n;
    }

    int find_nearest(Eigen::Vector2d query)
    {
        int index = -1;
        double min_d = std::numeric_limits<double>::max();
        find_nearest(root_, query, index, min_d);
        return index;
    }

private:
    void find_nearest(std::shared_ptr<Node> node, Eigen::Vector2d query, int& index, double& min_d)
    {
        if(node == nullptr){
            return;
        }
        // ROS_INFO_STREAM("node loc: " << node->location);
        const Eigen::Vector2d p = points_[node->location];
        const double d = (p - query).norm();
        if(d < min_d){
            min_d = d;
            index = node->location;
        }
        const int axis = node->axis;
        const double diff_axis = query(axis) - p(axis);
        if(diff_axis < 0){
            find_nearest(node->left_child, query, index, min_d);
        }else{
            find_nearest(node->right_child, query, index, min_d);
        }
        if(std::abs(diff_axis) < min_d){
            if(!(diff_axis < 0)){
                find_nearest(node->left_child, query, index, min_d);
            }else{
                find_nearest(node->right_child, query, index, min_d);
            }
        }
    }

    std::vector<Eigen::Vector2d> points_;
    int k_;
    std::shared_ptr<Node> root_;
};


}

}
#endif// __NODE_EDGE_LOCALIZER_KDTREE_H
