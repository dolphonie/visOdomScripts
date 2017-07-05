#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>


ros::Publisher pose_pub;

void odomCallback(const nav_msgs::Odometry& msg){
  geometry_msgs::PoseWithCovarianceStamped pose;
  pose.header = msg.header;
  pose.pose = msg.pose;

  pose_pub.publish(pose);
  ROS_DEBUG("Transform published");

}

int main(int argc, char** argv){
  ros::init(argc, argv, "odom_forwarder");
    
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("rtabmap/odom", 10, &odomCallback);
  ROS_INFO("Subscribed to odom");
  pose_pub = node.advertise<geometry_msgs::PoseWithCovarianceStamped>("mavros/vision_pose/pose_cov", 10);
  ROS_INFO("Published topic");
  ros::spin();
  return 0;
};
