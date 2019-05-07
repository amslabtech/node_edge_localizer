#include <node_edge_localizer/calculation.h>

namespace Calculation{
	double pi_2_pi(double angle)
	{
		return atan2(sin(angle), cos(angle));
	}

	void calculate_pca(std::vector<Eigen::Vector3d>& traj, Eigen::Vector2d& eigen_values, Eigen::Matrix2d& eigen_vectors)
	{
		// principal component analysis
		double size = traj.size();
		double ave_x = 0;
		double ave_y = 0;
		for(auto& point : traj){
			ave_x += point(0);	
			ave_y += point(1);	
		}
		ave_x /= size;
		ave_y /= size;
		double sigma_xx = 0;
		double sigma_xy = 0;
		double sigma_yy = 0;
		for(auto& point : traj){
			sigma_xx += (point(0) - ave_x) * (point(0) - ave_x); 
			sigma_xy += (point(0) - ave_x) * (point(1) - ave_y); 
			sigma_yy += (point(1) - ave_y) * (point(1) - ave_y); 
		}
		Eigen::Matrix2d cov_mat;
		cov_mat << sigma_xx, sigma_xy,
				   sigma_xy, sigma_yy; 
		Eigen::EigenSolver<Eigen::Matrix2d> es(cov_mat);
		eigen_values = es.eigenvalues().real();
		eigen_vectors = es.eigenvectors().real();
	}

	void calculate_pca(std::vector<Eigen::Vector3d>& traj, Eigen::Vector2d& eigen_values, Eigen::Matrix2d& eigen_vectors, Eigen::Vector2d& center)
	{
		// principal component analysis
		double size = traj.size();
		double ave_x = 0;
		double ave_y = 0;
		for(auto& point : traj){
			ave_x += point(0);	
			ave_y += point(1);	
		}
		ave_x /= size;
		ave_y /= size;
		double sigma_xx = 0;
		double sigma_xy = 0;
		double sigma_yy = 0;
		for(auto& point : traj){
			sigma_xx += (point(0) - ave_x) * (point(0) - ave_x); 
			sigma_xy += (point(0) - ave_x) * (point(1) - ave_y); 
			sigma_yy += (point(1) - ave_y) * (point(1) - ave_y); 
		}
		Eigen::Matrix2d cov_mat;
		cov_mat << sigma_xx, sigma_xy,
				   sigma_xy, sigma_yy; 
		Eigen::EigenSolver<Eigen::Matrix2d> es(cov_mat);
		eigen_values = es.eigenvalues().real();
		eigen_vectors = es.eigenvectors().real();
		center << ave_x, ave_y;
	}

	double square(double value)
	{
		return value * value;
	}

	double get_slope_angle(double angle)
	{
		if(angle > M_PI / 2.0){
			angle -= M_PI;
		}else if(angle < -M_PI / 2.0){
			angle += M_PI;
		}
		return angle;
	}

	double get_curvature_from_trajectory(std::vector<Eigen::Vector3d>& traj)
	{
		Eigen::Vector2d eigen_values;
		Eigen::Matrix2d eigen_vectors;
		calculate_pca(traj, eigen_values, eigen_vectors);
		double min_value = (eigen_values(0) < eigen_values(1)) ? eigen_values(0) : eigen_values(1); 
		double curvature = min_value / (eigen_values(0) + eigen_values(1));
		return curvature;
	}

	double get_angle_from_trajectory(std::vector<Eigen::Vector3d>& traj)
	{
		Eigen::Vector2d eigen_values;
		Eigen::Matrix2d eigen_vectors;
		calculate_pca(traj, eigen_values, eigen_vectors);
		double larger_index = (eigen_values(0) > eigen_values(1)) ? 0 : 1; 
		Eigen::Vector2d larger_vector = eigen_vectors.col(larger_index);
		double angle = atan2(larger_vector(1), larger_vector(0));
		return angle;
	}

	void get_slope_from_trajectory(std::vector<Eigen::Vector3d>& traj, Eigen::Vector2d& slope)
	{
		Eigen::Vector2d eigen_values;
		Eigen::Matrix2d eigen_vectors;
		calculate_pca(traj, eigen_values, eigen_vectors);
		double larger_index = (eigen_values(0) > eigen_values(1)) ? 0 : 1; 
		Eigen::Vector2d larger_vector = eigen_vectors.col(larger_index);
		// the first principal component
		slope = larger_vector;
	}

	void get_slope_and_center_from_trajectory(std::vector<Eigen::Vector3d>& traj, Eigen::Vector2d& slope, Eigen::Vector2d& center) 
	{
		Eigen::Vector2d eigen_values;
		Eigen::Matrix2d eigen_vectors;
		calculate_pca(traj, eigen_values, eigen_vectors, center);
		double larger_index = (eigen_values(0) > eigen_values(1)) ? 0 : 1; 
		Eigen::Vector2d larger_vector = eigen_vectors.col(larger_index);
		// the first principal component
		slope = larger_vector;
	}

	double get_angle_from_lines(Eigen::Vector3d& line0_p0, Eigen::Vector3d& line0_p1, Eigen::Vector3d& line1_p0, Eigen::Vector3d& line1_p1)
	{
		double v0_x = line0_p0(0) - line0_p1(0);
		double v0_y = line0_p0(1) - line0_p1(1);
		double v1_x = line1_p0(0) - line1_p1(0);
		double v1_y = line1_p0(1) - line1_p1(1);
		return acos((v0_x * v1_x + v0_y * v1_y) / (sqrt(v0_x * v0_x + v0_y * v0_y) * sqrt(v1_x * v1_x + v1_y * v1_y)));
	}

	double get_length_of_trajectory(std::vector<Eigen::Vector3d>& traj)
	{
		if(traj.empty()){
			return 0;
		}
		Eigen::Vector3d p0 = *(traj.begin());
		Eigen::Vector3d p1 = *(traj.end() - 1);
		Eigen::Vector3d diff = p1 - p0;
		return sqrt(square(diff(0)) + square(diff(1)));
	}

}
