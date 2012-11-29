#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>

void pointsCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  ROS_INFO("Point cloud callback");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "plane_fit");
  ros::NodeHandle nh;
  ros::Subscriber points_subscriber = nh.subscribe("/camera/points", 10, pointsCallback);
  ros::spin();
  return 0;
}
