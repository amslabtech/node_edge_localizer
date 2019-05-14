# node_edge_localizer
[![Build Status](https://travis-ci.org/amslabtech/node_edge_localizer.svg?branch=master)](https://travis-ci.org/amslabtech/node_edge_localizer)
![issue_opened](https://img.shields.io/github/issues/amslabtech/node_edge_localizer.svg)
![issue_closed](https://img.shields.io/github/issues-closed/amslabtech/node_edge_localizer.svg)

## Environment
- Ubuntu 16.04LTS
- ROS Kinetic

## Dependencies
- amsl_navigation_managers
- amsl_navigation_msgs

## Published topics
- /estimated_pose/pose (nav_msgs/Odometry)
  - estimated robot pose on map
- /estimated_pose/edge (amsl_navigation_msgs/Edge)
  - an edge presumed to have the robot
- /estimated_pose/particles
  - visualization of particle filter
## Subscribed topics
- /node_edge_map/map (amsl_navigation_msgs/NodeEdgeMap)
  - the node edge map
- /odom/complement
  - wheel odometry (complementation with IMU is recommended)

## Parameters
- hz
  - main loop rate (default: 20[Hz])
- init_node0_id
  - node id of the begin of initial edge (default: 0)
- init_node1_id
  - node id of the end of initial edge (default: 1)
- init_progress
  - initial progress on initial edge (default: 0.0)
- init_yaw
  - initial robot yaw in map frame (default: 0.0)
- curvature_threshold
  - trajectories with curvature greater than this parameter are considered as curves (default: 0.010)
- pose_num_pca
  - number of robot poses to calculate principal component analysis for calculating trajectory curvature and angle (default: 50)
- min_line_size
  - number of robot poses that exceed this parameter is considered a trajectory (default: 50)
    - this parameter is recommended to correspond with pose_num_pca
- min_line_length
  - trajectories that are longer than this parameter and aren't curve is considered as straight lines (default: 3.0[m])
- enable_tf
  - if this parameter is true, this node publishes transform from map frame to robot frame (default: false)
    - to publish transform, odom frame to robot frame tf is required 
- use_orientation_z_as_yaw
  - in principle, this parameter should NOT be set to "true" (default: false)
- particles_num
  - number of particles (default: 1000)
- noise_sigma
  - standard deviation of particle motion noise (default: 0.10)
- edge_decision_threshold
  - when the robot passes the node by this distance, the particles determine the next edge according to the robot's orientation (default: 0.5[m])
- same_trajectory_angle_threshold
  - the continuous trajectories with less orientation difference than this angle is considered to be same trajectory (default: M_PI/6[rad])
- continuous_line_threshold 
  - connected edges with angles smaller than the threshold are considered straight (default: M_PI/7[rad])
- line_edge_ratio_threshold (deprecated)
  - linear trajectories shorter than the threshold aren't used for correction (default: 0.5[m])
- enable_odom_tf
  - if this param is true, this node publishes internally calculated odometry (default: false)
- correction_rejection_angle_difference_threshold
  - if yaw of the correction result is greater than this threshold, the correction is rejected (default: M_PI/6[rad])
- resampling_interval
  - resampling particles once every (this param) times (default: 5)
