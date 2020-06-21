/**
 * @file localizer_node.cpp 
 * @author amsl
 */

#include "node_edge_localizer/localizer.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "localizer");
    node_edge_localizer::Localizer localizer;
    localizer.process();
    return 0;
}