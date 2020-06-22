#include <gtest/gtest.h>
#include <ros/ros.h>

#include "node_edge_localizer/localizer.h"

class OdomSubTest: public ::testing::Test
{
public:
    OdomSubTest(void)
    {
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 1, true);
        pose_sub_ = nh_.subscribe("estimated_pose", 1, &OdomSubTest::pose_callback, this, ros::TransportHints().reliable().tcpNoDelay(true));
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
        estimated_pose_updated_ = false;
        pose_ = nav_msgs::Odometry();
        count_ = 0;
    }

    geometry_msgs::Quaternion get_quaternion_msg_from_yaw(const double yaw)
    {
        tf2::Quaternion q;
        q.setEuler(0, 0, 0);
        return tf2::toMsg(q);
    }

    ros::NodeHandle nh_;
    ros::Publisher odom_pub_;
    ros::Subscriber pose_sub_;
    bool estimated_pose_updated_;
    unsigned int count_;
    nav_msgs::Odometry pose_;
};

TEST_F(OdomSubTest, StopCase)
{
    ros::Duration(1.0).sleep();
    nav_msgs::Odometry odom;
    odom.child_frame_id = "base_link";
    odom.header.frame_id = "odom";
    odom.header.stamp = ros::Time::now();
    odom.pose.pose.orientation = get_quaternion_msg_from_yaw(0);
    odom_pub_.publish(odom);
    ros::Duration(0.1).sleep();

    while(ros::ok()){
        std::cout << "loop..." << std::endl;
        if(count_ > 10) break;
        odom.header.stamp = ros::Time::now();
        odom_pub_.publish(odom);
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
    ASSERT_LT(abs(pose_.pose.pose.position.x - odom.pose.pose.position.x), 0.01);
    ASSERT_LT(abs(pose_.pose.pose.position.y - odom.pose.pose.position.y), 0.01);
    double angle_diff = tf2::getYaw(pose_.pose.pose.orientation) - tf2::getYaw(odom.pose.pose.orientation);
    ASSERT_LT(abs(angle_diff), 0.01);
}

TEST_F(OdomSubTest, LinearCase)
{
    ros::Duration(1.0).sleep();
    nav_msgs::Odometry odom;
    odom.child_frame_id = "base_link";
    odom.header.frame_id = "odom";
    odom.header.stamp = ros::Time::now();
    odom.pose.pose.orientation = get_quaternion_msg_from_yaw(0);
    odom_pub_.publish(odom);
    ros::Duration(0.1).sleep();

    while(ros::ok()){
        std::cout << "loop..." << std::endl;
        if(count_ > 10) break;
        odom.header.stamp = ros::Time::now();
        odom.pose.pose.position.x += 0.01;
        odom_pub_.publish(odom);
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
    ASSERT_LT(abs(pose_.pose.pose.position.x - odom.pose.pose.position.x), 0.01);
    ASSERT_LT(abs(pose_.pose.pose.position.y - odom.pose.pose.position.y), 0.01);
    double angle_diff = tf2::getYaw(pose_.pose.pose.orientation) - tf2::getYaw(odom.pose.pose.orientation);
    ASSERT_LT(abs(angle_diff), 0.01);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "intersection_detection_test");
    return RUN_ALL_TESTS();
}