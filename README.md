# node_edge_localizer
![issue_opened](https://img.shields.io/github/issues/amslabtech/node_edge_localizer.svg)
![issue_closed](https://img.shields.io/github/issues-closed/amslabtech/node_edge_localizer.svg)

## dependencies
- amsl_navigation_managers
- amsl_navigation_msgs

## published topics
- /estimated_pose/odom (nav_msgs/Odometry)
  - estimated robot pose on map
- /estimated_pose/edge (amsl_navigation_msgs/Edge)
  - an edge presumed to have the robot
- /estimated_pose/particles
  - visualization of particle filter
## subscribed topics
- /node_edge_map (amsl_navigation_msgs/NodeEdgeMap)
  - the node edge map
- /odom/complement
  - wheel odometry (complementation with IMU is recommended)
