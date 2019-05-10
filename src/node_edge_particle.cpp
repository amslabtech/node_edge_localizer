#include "node_edge_localizer/node_edge_particle.h"

NodeEdgeParticle::NodeEdgeParticle(void)
	:x(0.0), y(0.0), yaw(0.0),
	 current_edge_index(-1), last_edge_index(-1), 
	 robot_distance_from_last_node(0.0),
	 last_node_x(0.0), last_node_y(0.0),
	 weight(0.0), near_node_flag(false)
{

}

double NodeEdgeParticle::get_distance_from_last_node(double x, double y)
{
	return sqrt(square(last_node_x - x) + square(last_node_y - y));
}

double NodeEdgeParticle::get_particle_distance_from_last_node(void)
{
	return sqrt(square(last_node_x - x) + square(last_node_y - y));
}

double NodeEdgeParticle::square(double value)
{
	return value * value;
}

void NodeEdgeParticle::move(double distance, double direction)
{
	yaw = direction;
	x += distance * cos(yaw);
	y += distance * sin(yaw);
}

void NodeEdgeParticle::evaluate(double robot_orientation)
{
	double diff = yaw - robot_orientation;	
	diff = fabs(atan2(sin(diff), cos(diff)));
	weight = (1 - diff / M_PI);

	weight *= weight;

	if(weight < MIN_WEIGHT){
		weight = MIN_WEIGHT;
	}
}
