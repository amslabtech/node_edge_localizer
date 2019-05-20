#include <gtest/gtest.h>

#include <ros/ros.h>

#include <node_edge_localizer/calculation.h>

TEST(TestSuite, test_calculation1)
{
	ros::NodeHandle nh;
	EXPECT_EQ(Calculation::square(M_PI), M_PI * M_PI);
}

int main(int argc, char** argv)
{
	testing::InitGoogleTest(&argc, argv);

	ros::init(argc, argv, "node_edge_localizer_test");

	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::Duration(3.0).sleep();

	int r_e_t = RUN_ALL_TESTS();

	spinner.stop();

	ros::shutdown();

	return r_e_t;
}
