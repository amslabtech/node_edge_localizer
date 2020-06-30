#include <gtest/gtest.h>
#include <ros/ros.h>

#include "amsl_navigation_msgs/NodeEdgeMap.h"
#include "node_edge_localizer/localizer.h"

class ObservationTest: public ::testing::Test
{
public:
    ObservationTest(void)
    {
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 1, true);
        init_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1, true);
        observation_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("observation_map", 1, true);
        pose_sub_ = nh_.subscribe("estimated_pose/pose", 1, &ObservationTest::pose_callback, this, ros::TransportHints().reliable().tcpNoDelay(true));
    }

    void pose_callback(const nav_msgs::OdometryConstPtr& msg)
    {
        estimated_pose_updated_ = true;
        pose_ = *msg;
        count_++;
    }

protected:
    virtual void SetUp()
    {
        // wait for localizer receives the map
        ros::Duration(1.0).sleep();
        estimated_pose_updated_ = false;
        pose_ = nav_msgs::Odometry();
        count_ = 0;
        geometry_msgs::PoseWithCovarianceStamped init_pose;
        init_pose.header.frame_id = "map";
        init_pose.pose.pose.orientation = get_quaternion_msg_from_yaw(0.0);
        init_pose_pub_.publish(init_pose);
        ros::Duration(0.5).sleep();
    }

    geometry_msgs::Quaternion get_quaternion_msg_from_yaw(const double yaw)
    {
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        return tf2::toMsg(q);
    }

    nav_msgs::OccupancyGrid get_observation_map(void)
    {
        nav_msgs::OccupancyGrid observation_map;
        observation_map.header.frame_id = "base_link";
        observation_map.header.stamp = ros::Time::now();
        observation_map.info.width = 100;
        observation_map.info.height = 100;
        observation_map.info.resolution = 0.10;
        observation_map.info.origin.position.x = -5.0;
        observation_map.info.origin.position.y = -5.0;
        const unsigned int size = observation_map.info.width * observation_map.info.height;
        observation_map.data.resize(size);
        for(auto& d : observation_map.data){
            d = -1;
        }
        for(unsigned int i=0;i<size;i++){
            // const unsigned ix = i % observation_map.info.width;
            const unsigned iy = floor(i / observation_map.info.width);
            if(20 < iy && iy < 80){
                observation_map.data[i] = 0;
            }else if(iy == 20 || iy == 80){
                observation_map.data[i] = 100;
            }
        }
        return observation_map;
    }

    ros::NodeHandle nh_;
    ros::Publisher odom_pub_;
    ros::Publisher init_pose_pub_;
    ros::Publisher observation_map_pub_;
    ros::Subscriber pose_sub_;
    bool estimated_pose_updated_;
    unsigned int count_;
    nav_msgs::Odometry pose_;
};

TEST_F(ObservationTest, TestCase)
{
    nav_msgs::Odometry odom;
    odom.child_frame_id = "base_link";
    odom.header.frame_id = "odom";
    odom.header.stamp = ros::Time::now();
    odom.pose.pose.orientation = get_quaternion_msg_from_yaw(0);
    odom_pub_.publish(odom);
    ros::Duration(0.1).sleep();

    while(ros::ok()){
        std::cout << "loop..." << std::endl;
        odom.header.stamp = ros::Time::now();
        const double dx = 0.1;
        odom.pose.pose.position.x += dx;
        odom.pose.pose.position.y = 0;;
        odom.pose.pose.orientation = get_quaternion_msg_from_yaw(0);
        std::cout << "yaw: " << tf2::getYaw(odom.pose.pose.orientation) << std::endl;;
        odom_pub_.publish(odom);
        observation_map_pub_.publish(get_observation_map());
        ros::spinOnce();
        ros::Duration(0.1).sleep();
        if(count_ > 100){
            ros::spinOnce();
            break;
        }
    }
    ros::spinOnce();
    std::cout << "odom:\n" << odom.pose.pose << std::endl;
    std::cout << "pose:\n" << pose_.pose.pose << std::endl;
    ASSERT_LT(sqrt(pow(pose_.pose.pose.position.x - odom.pose.pose.position.x, 2) + pow(pose_.pose.pose.position.y - odom.pose.pose.position.y, 2)), 0.10);
    double angle_diff = tf2::getYaw(pose_.pose.pose.orientation) - tf2::getYaw(odom.pose.pose.orientation);
    ASSERT_LT(abs(angle_diff), 0.05);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "observation_test");
    return RUN_ALL_TESTS();
}