#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistWithCovariance.h>

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // 打印里程计消息的头信息
    ROS_INFO("Header: seq: %d, stamp: %f, frame_id: %s",
             msg->header.seq, msg->header.stamp.toSec(), msg->header.frame_id.c_str());

    // 打印子坐标系 (child_frame_id)
    ROS_INFO("Child Frame ID: %s", msg->child_frame_id.c_str());

    // 解析位姿 (PoseWithCovariance)
    const geometry_msgs::PoseWithCovariance& pose = msg->pose;
    ROS_INFO("Pose Position: [x: %f, y: %f, z: %f]", 
             pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
    ROS_INFO("Pose Orientation: [x: %f, y: %f, z: %f, w: %f]",
             pose.pose.orientation.x, pose.pose.orientation.y, 
             pose.pose.orientation.z, pose.pose.orientation.w);

    // 解析速度 (TwistWithCovariance)
    const geometry_msgs::TwistWithCovariance& twist = msg->twist;
    ROS_INFO("Twist Linear: [x: %f, y: %f, z: %f]", 
             twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z);
    ROS_INFO("Twist Angular: [x: %f, y: %f, z: %f]", 
             twist.twist.angular.x, twist.twist.angular.y, twist.twist.angular.z);
}

int main(int argc, char** argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "odom_listener");

    // 创建节点句柄
    ros::NodeHandle nh;

    // 订阅里程计话题
    ros::Subscriber odom_sub = nh.subscribe("/Odometry", 10, odomCallback);

    // 循环等待回调函数
    ros::spin();

    return 0;
}
