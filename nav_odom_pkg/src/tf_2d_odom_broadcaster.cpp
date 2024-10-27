#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_2d_odom_broadcaster");

  ros::NodeHandle node;

  tf::TransformListener listener;
  tf::TransformBroadcaster broadcaster;
  tf::Transform transform_broadcaster;

  // 创建一个Publisher来发布Odometry消息
  ros::Publisher odom_pub = node.advertise<nav_msgs::Odometry>("body_2d_odom", 10);

  ros::Duration(1.0).sleep();
  ros::Rate rate(1000);

  while (node.ok()){
    tf::StampedTransform transform_listener;
    
    try{
      listener.lookupTransform("map", "body",  
                               ros::Time(0), transform_listener);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;  // 如果有异常，跳过本次循环
    }

    // 获取机器人的位置和方向
    float robot_pose_x = transform_listener.getOrigin().x();
    float robot_pose_y = transform_listener.getOrigin().y();
    float robot_pose_z = 0;  // 只考虑二维，z设置为0
    float robot_oriation_z = transform_listener.getRotation().getZ();
    float robot_oriation_w = transform_listener.getRotation().getW();

    // 发布TF，map到body_2d
    transform_broadcaster.setOrigin( tf::Vector3(robot_pose_x, robot_pose_y, 0.0) );
    transform_broadcaster.setRotation( tf::Quaternion(0, 0, robot_oriation_z, robot_oriation_w) );
    broadcaster.sendTransform(tf::StampedTransform(transform_broadcaster, ros::Time::now(), "map", "body_2d"));

    // 创建Odometry消息
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "map";  // 发布在map坐标系下

    // 设置位置 (x, y, z)
    odom.pose.pose.position.x = robot_pose_x;
    odom.pose.pose.position.y = robot_pose_y;
    odom.pose.pose.position.z = 0.0;

    // 设置方向 (仅考虑平面上的z轴旋转)
    odom.pose.pose.orientation.z = robot_oriation_z;
    odom.pose.pose.orientation.w = robot_oriation_w;

    // 发布Odometry消息
    odom_pub.publish(odom);

    rate.sleep();
  }
  return 0;
}
