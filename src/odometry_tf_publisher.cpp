#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <string>

ros::Publisher odometry_pub; 
ros::Subscriber odom_pose2D_sub; 
ros::Subscriber odom_vel_sub;

geometry_msgs::Pose2D odom_pose2D;
geometry_msgs::TwistStamped odom_vel;
nav_msgs::Odometry odom;
geometry_msgs::TransformStamped odom_tf;

std::string odom_child_frame_id;

auto createQuaternionMsgFromYaw(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return tf2::toMsg(q);
}

void odomPose2DCallback(const geometry_msgs::Pose2D& odom_pose2D)
{
    odom.pose.pose.position.x = odom_pose2D.x;
    odom.pose.pose.position.y = odom_pose2D.y;
    odom.pose.pose.position.z = 0;
    odom.pose.pose.orientation = createQuaternionMsgFromYaw(odom_pose2D.theta);
    odom_tf.transform.translation.x = odom.pose.pose.position.x;
    odom_tf.transform.translation.y = odom.pose.pose.position.y;
    odom_tf.transform.translation.z = odom.pose.pose.position.z;
    odom_tf.transform.rotation      = odom.pose.pose.orientation;
}

void odomVelCallback(const geometry_msgs::TwistStamped& odom_velocity)
{
    static tf2_ros::TransformBroadcaster br;
    odom.header.stamp = odom_velocity.header.stamp;
    odom.header.frame_id = odom_velocity.header.frame_id;
    odom.child_frame_id  = odom_child_frame_id;
    odom_tf.header = odom.header;
    odom_tf.child_frame_id = odom.child_frame_id;

    odom.twist.twist.linear.x  = odom_velocity.twist.linear.x;
    odom.twist.twist.angular.z = odom_velocity.twist.angular.z;
    odometry_pub.publish(odom);
    br.sendTransform(odom_tf);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odometry_tf_publisher");
    ros::NodeHandle nh;
    ROS_INFO("ODOMETRY TF PUBLISHER NODE");
    nh.getParam("/odometry_tf_publisher/odom_child_frame_id", odom_child_frame_id);

    odometry_pub = nh.advertise<nav_msgs::Odometry>("odometry", 1);
    odom_pose2D_sub = nh.subscribe("odom_pose2D", 1, odomPose2DCallback);
    odom_vel_sub = nh.subscribe("odom_velocity", 1, odomVelCallback);
    ros::spin();
    return 0;
}