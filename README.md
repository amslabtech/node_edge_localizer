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
- HZ
  - main loop rate (default: 20[Hz])
- INIT_NODE0_ID
  - node id of the begin of initial edge (default: 0)
- INIT_NODE1_ID
  - node id of the end of initial edge (default: 1)
- INIT_PROGRESS
  - initial progress on initial edge (default: 0.0)
- INIT_YAW
  - initial robot yaw in map frame (default: 0.0)
- CURVATURE_THRESHOLD
  - trajectories with curvature greater than this parameter are considered as curves (default: 0.010)
- POSE_NUM_PCA
  - number of robot poses to calculate principal component analysis for calculating trajectory curvature and angle (default: 50)
- MIN_LINE_SIZE
  - number of robot poses that exceed this parameter is considered a trajectory (default: 50)
    - this parameter is recommended to correspond with pose_num_pca
- MIN_LINE_LENGTH
  - trajectories that are longer than this parameter and aren't curve is considered as straight lines (default: 3.0[m])
- ENABLE_TF
  - if this parameter is true, this node publishes transform from map frame to robot frame (default: false)
    - to publish transform, odom frame to robot frame tf is required
- USE_ORIENTATION_Z_AS_YAW
  - in principle, this parameter should NOT be set to "true" (default: false)
- PARTICLES_NUM
  - number of particles (default: 1000)
- NOISE_SIGMA
  - standard deviation of particle motion noise (default: 0.10)
- EDGE_DECISION_THRESHOLD
  - when the robot passes the node by this distance, the particles determine the next edge according to the robot's orientation (default: 0.5[m])
- SAME_TRAJECTORY_ANGLE_THRESHOLD
  - the continuous trajectories with less orientation difference than this angle is considered to be same trajectory (default: M_PI/6[rad])
- CONTINUOUS_LINE_THRESHOLD
  - connected edges with angles smaller than the threshold are considered straight (default: M_PI/7[rad])
- LINE_EDGE_RATIO_THRESHOLD (DEPRECATED)
  - linear trajectories shorter than the threshold aren't used for correction (default: 0.5[m])
- ENABLE_ODOM_TF
  - if this param is true, this node publishes internally calculated odometry (default: false)
- CORRECTION_REJECTION_ANGLE_DIFFERENCE_THRESHOLD
  - if yaw of the correction result is greater than this threshold, the correction is rejected (default: M_PI/6[rad])
- RESAMPLING_INTERVAL
  - resampling particles once every (this param) times (default: 5)
- EDGE_CERTAIN_THRESHOLD
  - particles ratio for deciding estimated edge (default: 0.9)
