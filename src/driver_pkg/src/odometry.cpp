#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

double vx = 0.0;
double vy = 0.0;
double vth = 0.0;

float steering_angle_;
float linear_velocity_x_;
float linear_velocity_y_;
float angular_velocity_z_;
ros::Time last_vel_time_;
float vel_dt_;
float x_pos_;
float y_pos_;
float heading_;

tf2::Quaternion odom_quat;
geometry_msgs::TransformStamped odom_trans;
nav_msgs::Odometry odom;
ros::Publisher odom_pub;
ros::Subscriber vels_sub;
ros::Time current_time, last_time;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "odometry_publisher");

    ros::NodeHandle n;
    void velCallback(const geometry_msgs::Twist &vel);
    vels_sub = n.subscribe("/velocities", 1000, velCallback);
    odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);

    double x = 0.0;
    double y = 0.0;
    double th = 0.0;

    
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    // ROS_WARN("START ODODM");
    while(n.ok())
    {
        ros::spinOnce();
    }
}

void velCallback(const geometry_msgs::Twist &vel)
{
    current_time = ros::Time::now();

    linear_velocity_x_ = vel.linear.x;
    linear_velocity_y_ = vel.linear.y;
    angular_velocity_z_ = vel.angular.z;

    vel_dt_ = (current_time - last_vel_time_).toSec();
    last_vel_time_ = current_time;

    double delta_heading = angular_velocity_z_ * vel_dt_;                                                 //radians
    double delta_x = (linear_velocity_x_ * cos(heading_) - linear_velocity_y_ * sin(heading_)) * vel_dt_; //m
    double delta_y = (linear_velocity_x_ * sin(heading_) + linear_velocity_y_ * cos(heading_)) * vel_dt_; //m

    //calculate current position of the robot
    x_pos_ += delta_x;
    y_pos_ += delta_y;
    heading_ += delta_heading;

    //calculate robot's heading in quaternion angle
    //ROS has a function to calculate yaw in quaternion angle
    odom_quat.setRPY(0, 0, heading_);

    odom_trans.header.frame_id = "odom";
    odom_trans.header.stamp = current_time;
    odom_trans.child_frame_id = "base_link";
    //robot's position in x,y, and z
    odom_trans.transform.translation.x = x_pos_;
    odom_trans.transform.translation.y = y_pos_;
    odom_trans.transform.translation.z = 0.0;
    //robot's heading in quaternion
    odom_trans.transform.rotation.x = odom_quat.x();
    odom_trans.transform.rotation.y = odom_quat.y();
    odom_trans.transform.rotation.z = odom_quat.z();
    odom_trans.transform.rotation.w = odom_quat.w();
    odom_trans.header.stamp = current_time;
    //publish robot's tf using odom_trans object
    //odom_broadcaster_.sendTransform(odom_trans);

    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    //robot's position in x,y, and z
    odom.pose.pose.position.x = x_pos_;
    odom.pose.pose.position.y = y_pos_;
    odom.pose.pose.position.z = 0.0;
    //robot's heading in quaternion
    odom.pose.pose.orientation.x = odom_quat.x();
    odom.pose.pose.orientation.y = odom_quat.y();
    odom.pose.pose.orientation.z = odom_quat.z();
    odom.pose.pose.orientation.w = odom_quat.w();
    odom.pose.covariance[0] = 0.001;
    odom.pose.covariance[7] = 0.001;
    odom.pose.covariance[35] = 0.001;

    //linear speed from encoders
    odom.twist.twist.linear.x = linear_velocity_x_;
    odom.twist.twist.linear.y = linear_velocity_y_;
    odom.twist.twist.linear.z = 0.0;

    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    //angular speed from encoders
    odom.twist.twist.angular.z = angular_velocity_z_;
    odom.twist.covariance[0] = 0.0001;
    odom.twist.covariance[7] = 0.0001;
    odom.twist.covariance[35] = 0.0001;

    odom_pub.publish(odom);
}