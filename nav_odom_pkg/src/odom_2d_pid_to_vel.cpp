#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <algorithm>
#include <geometry_msgs/Twist.h>
#include <cmath>

class PID {
public:
    enum Mode {
        PID_POSITION = 0,
        PID_DELTA
    };

    PID(Mode mode, const float* PID_values, float max_out, float max_iout) 
        : mode(mode), Kp(PID_values[0]), Ki(PID_values[1]), Kd(PID_values[2]), 
          max_out(max_out), max_iout(max_iout), 
          set(0.0f), fdb(0.0f), 
          out(0.0f), Pout(0.0f), Iout(0.0f), Dout(0.0f) {
        std::fill_n(Dbuf, 3, 0.0f);
        std::fill_n(error, 3, 0.0f);
    }

    float calc(float ref, float set) {
        error[2] = error[1];
        error[1] = error[0];
        this->set = set;
        this->fdb = ref;
        error[0] = set - ref;

        Pout = Kp * error[0];
        Iout += Ki * error[0];
        Dbuf[2] = Dbuf[1];
        Dbuf[1] = Dbuf[0];
        Dbuf[0] = (error[0] - error[1]);
        Dout = Kd * Dbuf[0];
        out = Pout + Iout + Dout;

        // 限制输出
        if (out > max_out) out = max_out;
        if (out < -max_out) out = -max_out;

        return out;
    }

    void clear() {
        std::fill_n(error, 3, 0.0f);
        std::fill_n(Dbuf, 3, 0.0f);
        out = Pout = Iout = Dout = 0.0f;
        fdb = set = 0.0f;
    }

private:
    Mode mode;
    float Kp, Ki, Kd;
    float max_out;
    float max_iout;

    float set;
    float fdb;

    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];
    float error[3];
};

// 定义三个PID控制器用于控制x, y 和 yaw
float pid_x_values[3] = {1.0f, 0.0f, 0.1f}; // PID参数
float pid_y_values[3] = {1.0f, 0.0f, 0.1f};
float pid_yaw_values[3] = {1.0f, 0.0f, 0.1f};

PID pid_x(PID::PID_POSITION, pid_x_values, 1.0f, 0.1f);   // PID for x direction
PID pid_y(PID::PID_POSITION, pid_y_values, 1.0f, 0.1f);   // PID for y direction
PID pid_yaw(PID::PID_POSITION, pid_yaw_values, 1.0f, 0.1f); // PID for yaw

// 目标位置和角度
double target_x = 1.0;   // 目标x位置
double target_y = 1.0;   // 目标y位置
double target_yaw = 1.57; // 目标yaw角 (弧度)

ros::Publisher pub;

// 里程计回调函数
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;

    double roll, pitch, yaw;
    tf::Quaternion quat(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    // 1. 计算目标点相对于机器人的平移量
    double delta_x = target_x - x;
    double delta_y = target_y - y;

    // 2. 将目标点从全局坐标系转换到机器人底盘坐标系
    double target_x_chassis = delta_x * std::cos(-yaw) - delta_y * std::sin(-yaw);
    double target_y_chassis = delta_x * std::sin(-yaw) + delta_y * std::cos(-yaw);

    // 3. 使用PID控制器计算速度
    float x_vel = pid_x.calc(0, target_x_chassis);
    float y_vel = pid_y.calc(0, target_y_chassis);
    float yaw_vel = pid_yaw.calc(yaw, target_yaw);

    ROS_INFO("Calculated velocities -> x: %f, y: %f, yaw: %f", x_vel, y_vel, yaw_vel);

    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = x_vel;
    vel_msg.linear.y = y_vel;
    vel_msg.linear.z = 0;
    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    vel_msg.angular.z = yaw_vel;
    
    pub.publish(vel_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_2d_pid_to_vel");
    ros::NodeHandle nh;

    ros::Subscriber odom_sub = nh.subscribe("/body_2d_odom", 10, odomCallback);

    pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",10);

    ros::spin();
    return 0;
}
