#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <std_msgs/Float32.h>

ros::Publisher battery_state_pub;
ros::Subscriber battery_voltage_sub;

sensor_msgs::BatteryState battery_state_msg;
double capacity_current;
double capacity_voltage;

void batteryVoltageCallback(const std_msgs::Float32& bat_voltage)
{
	battery_state_msg.header.stamp = ros::Time::now();
	battery_state_msg.capacity = capacity_current;
	battery_state_msg.voltage = bat_voltage.data;
	battery_state_msg.percentage = bat_voltage.data/capacity_voltage;
	battery_state_pub.publish(battery_state_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "battery_state_publisher");
    ros::NodeHandle nh;
    ROS_INFO("BATTERY STATE PUBLISHER NODE");
    nh.getParam("/battery_state_publisher/capacity_current", capacity_current);
    nh.getParam("/battery_state_publisher/capacity_voltage", capacity_voltage);

    battery_state_pub = nh.advertise<sensor_msgs::BatteryState>("battery_state", 1);
    battery_voltage_sub = nh.subscribe("battery_voltage", 1, batteryVoltageCallback);
    ros::spin();
    return 0;
}