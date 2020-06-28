#include <gtest/gtest.h>
#include <ros/ros.h>

#include "amsl_navigation_msgs/NodeEdgeMap.h"
#include "node_edge_localizer/localizer.h"

class LocalizationTest: public ::testing::Test
{
public:
    LocalizationTest(void)
    {
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 1, true);
        init_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1, true);
        pose_sub_ = nh_.subscribe("estimated_pose/pose", 1, &LocalizationTest::pose_callback, this, ros::TransportHints().reliable().tcpNoDelay(true));
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

    ros::NodeHandle nh_;
    ros::Publisher odom_pub_;
    ros::Publisher init_pose_pub_;
    ros::Subscriber pose_sub_;
    bool estimated_pose_updated_;
    unsigned int count_;
    nav_msgs::Odometry pose_;
};

TEST_F(LocalizationTest, StopCase)
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
        odom_pub_.publish(odom);
        ros::spinOnce();
        ros::Duration(0.1).sleep();
        if(count_ > 10){
            ros::spinOnce();
            break;
        }
    }
    ASSERT_LT(sqrt(pow(pose_.pose.pose.position.x - odom.pose.pose.position.x, 2) + pow(pose_.pose.pose.position.y - odom.pose.pose.position.y, 2)), 0.05);
    double angle_diff = tf2::getYaw(pose_.pose.pose.orientation) - tf2::getYaw(odom.pose.pose.orientation);
    ASSERT_LT(abs(angle_diff), 0.01);
}

TEST_F(LocalizationTest, LinearCase)
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
        odom.pose.pose.position.x += 0.1;
        odom_pub_.publish(odom);
        ros::spinOnce();
        ros::Duration(0.1).sleep();
        if(count_ > 10){
            ros::spinOnce();
            break;
        }
    }
    ros::spinOnce();
    std::cout << "odom:\n" << odom.pose.pose << std::endl;
    std::cout << "pose:\n" << pose_.pose.pose << std::endl;
    ASSERT_LT(sqrt(pow(pose_.pose.pose.position.x - odom.pose.pose.position.x, 2) + pow(pose_.pose.pose.position.y - odom.pose.pose.position.y, 2)), 0.05);
    double angle_diff = tf2::getYaw(pose_.pose.pose.orientation) - tf2::getYaw(odom.pose.pose.orientation);
    ASSERT_LT(abs(angle_diff), 0.01);
}

TEST_F(LocalizationTest, LeftCurveCase)
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
        odom.pose.pose.position.x += dx * cos(tf2::getYaw(odom.pose.pose.orientation));
        odom.pose.pose.position.y += dx * sin(tf2::getYaw(odom.pose.pose.orientation));
        odom.pose.pose.orientation = get_quaternion_msg_from_yaw(tf2::getYaw(odom.pose.pose.orientation) + 0.1);
        std::cout << "yaw: " << tf2::getYaw(odom.pose.pose.orientation) << std::endl;;
        odom_pub_.publish(odom);
        ros::spinOnce();
        ros::Duration(0.1).sleep();
        if(count_ > 10){
            ros::spinOnce();
            break;
        }
    }
    ros::spinOnce();
    std::cout << "odom:\n" << odom.pose.pose << std::endl;
    std::cout << "pose:\n" << pose_.pose.pose << std::endl;
    ASSERT_LT(sqrt(pow(pose_.pose.pose.position.x - odom.pose.pose.position.x, 2) + pow(pose_.pose.pose.position.y - odom.pose.pose.position.y, 2)), 0.05);
    double angle_diff = tf2::getYaw(pose_.pose.pose.orientation) - tf2::getYaw(odom.pose.pose.orientation);
    ASSERT_LT(abs(angle_diff), 0.01);
}

TEST_F(LocalizationTest, RightCurveCase)
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
        odom.pose.pose.position.x += dx * cos(tf2::getYaw(odom.pose.pose.orientation));
        odom.pose.pose.position.y += dx * sin(tf2::getYaw(odom.pose.pose.orientation));
        odom.pose.pose.orientation = get_quaternion_msg_from_yaw(tf2::getYaw(odom.pose.pose.orientation) - 0.1);
        std::cout << "yaw: " << tf2::getYaw(odom.pose.pose.orientation) << std::endl;;
        odom_pub_.publish(odom);
        ros::spinOnce();
        ros::Duration(0.1).sleep();
        if(count_ > 10){
            ros::spinOnce();
            break;
        }
    }
    ros::spinOnce();
    std::cout << "odom:\n" << odom.pose.pose << std::endl;
    std::cout << "pose:\n" << pose_.pose.pose << std::endl;
    ASSERT_LT(sqrt(pow(pose_.pose.pose.position.x - odom.pose.pose.position.x, 2) + pow(pose_.pose.pose.position.y - odom.pose.pose.position.y, 2)), 0.05);
    double angle_diff = tf2::getYaw(pose_.pose.pose.orientation) - tf2::getYaw(odom.pose.pose.orientation);
    ASSERT_LT(abs(angle_diff), 0.01);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "intersection_detection_test");
    return RUN_ALL_TESTS();
}