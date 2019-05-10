#ifndef __NODE_EDGE_PARTICLE_H
#define __NODE_EDGE_PARTICLE_H

#include <ros/ros.h>

#include "Eigen/Dense"
#include "Eigen/Geometry"

class NodeEdgeParticle
{
public:
	NodeEdgeParticle(void);

	double get_distance_from_last_node(double, double);
	double get_particle_distance_from_last_node(void);
	double square(double);
	void move(double, double);
	void evaluate(double);

	static constexpr double MIN_WEIGHT = 1e-5;

	double x;
	double y;
	// edge orientation
	double yaw;
	int current_edge_index;
	int last_edge_index;
	double robot_distance_from_last_node;
	double last_node_x;
	double last_node_y;
	double weight;
	bool near_node_flag;

private:
};

#endif// __NODE_EDGE_PARTICLE_H
