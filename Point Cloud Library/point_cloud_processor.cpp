#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "point_cloud_processor");
  ros::NodeHandle nh;

  ROS_INFO("Point cloud processor node is running...");
  ros::spin();
  return 0;
}