#ifndef __CALCULATION_H
#define __CALCULATION_H

#include <ros/ros.h>

#include "Eigen/Dense"

namespace Calculation{
    double pi_2_pi(double);
    void calculate_pca(std::vector<Eigen::Vector3d>&, Eigen::Vector2d&, Eigen::Matrix2d&);
    void calculate_pca(std::vector<Eigen::Vector3d>&, Eigen::Vector2d&, Eigen::Matrix2d&, Eigen::Vector2d&);
    double square(double);
    double get_slope_angle(double);
    double get_curvature_from_trajectory(std::vector<Eigen::Vector3d>&);
    double get_angle_from_trajectory(std::vector<Eigen::Vector3d>&);
    void get_slope_from_trajectory(std::vector<Eigen::Vector3d>&, Eigen::Vector2d&);
    void get_slope_and_center_from_trajectory(std::vector<Eigen::Vector3d>&, Eigen::Vector2d&, Eigen::Vector2d&);
    double get_angle_from_lines(Eigen::Vector3d&, Eigen::Vector3d&, Eigen::Vector3d&, Eigen::Vector3d&);
    double get_length_of_trajectory(std::vector<Eigen::Vector3d>&);
}

#endif // __CALCULATION_H
